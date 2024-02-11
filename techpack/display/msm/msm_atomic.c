/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <drm/drm_panel.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_kms.h"
#include "sde_trace.h"
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00028 */
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <video/mipi_display.h>
#include "sde/sde_encoder_phys.h"
#include "sharp/drm_cmn.h"
#include "sharp/drm_det.h"
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
#include "sde/sde_kms.h"
#include "dsi/dsi_display.h"
#include "dsi/dsi_panel.h"
#endif /* CONFIG_SHARP_DISPLAY */

#define MULTIPLE_CONN_DETECTED(x) (x > 1)

struct msm_commit {
	struct drm_device *dev;
	struct drm_atomic_state *state;
	uint32_t crtc_mask;
	uint32_t plane_mask;
	bool nonblock;
	struct kthread_work commit_work;
};

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
void msm_atomic_boost_commit_pending(struct msm_drm_private *priv);
#endif /* CONFIG_SHARP_DISPLAY */

static inline bool _msm_seamless_for_crtc(struct drm_device *dev,
					struct drm_atomic_state *state,
			struct drm_crtc_state *crtc_state, bool enable)
{
	struct drm_connector *connector = NULL;
	struct drm_connector_state  *conn_state = NULL;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int i = 0;
	int conn_cnt = 0;
	bool splash_en = false;

	if (msm_is_mode_seamless(&crtc_state->mode) ||
		msm_is_mode_seamless_vrr(&crtc_state->adjusted_mode) ||
		msm_is_mode_seamless_poms(&crtc_state->adjusted_mode) ||
		msm_is_mode_seamless_dyn_clk(&crtc_state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dms(&crtc_state->adjusted_mode) && !enable)
		return true;

	if (!crtc_state->mode_changed && crtc_state->connectors_changed) {
		for_each_old_connector_in_state(state, connector,
				conn_state, i) {
			if ((conn_state->crtc == crtc_state->crtc) ||
					(connector->state->crtc ==
					 crtc_state->crtc))
				conn_cnt++;

			if (kms && kms->funcs && kms->funcs->check_for_splash)
				splash_en = kms->funcs->check_for_splash(kms,
							 crtc_state->crtc);

			if (MULTIPLE_CONN_DETECTED(conn_cnt) && !splash_en)
				return true;
		}
	}

	return false;
}

static inline bool _msm_seamless_for_conn(struct drm_connector *connector,
		struct drm_connector_state *old_conn_state, bool enable)
{
	if (!old_conn_state || !old_conn_state->crtc)
		return false;

	if (!old_conn_state->crtc->state->mode_changed &&
			!old_conn_state->crtc->state->active_changed &&
			old_conn_state->crtc->state->connectors_changed) {
		if (old_conn_state->crtc == connector->state->crtc)
			return true;
	}

	if (enable)
		return false;

	if (!connector->state->crtc &&
		old_conn_state->crtc->state->connectors_changed)
		return false;

	if (msm_is_mode_seamless(&connector->encoder->crtc->state->mode))
		return true;

	if (msm_is_mode_seamless_vrr(
			&connector->encoder->crtc->state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dyn_clk(
			 &connector->encoder->crtc->state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dms(
			&connector->encoder->crtc->state->adjusted_mode))
		return true;

	return false;
}

/* clear specified crtcs (no longer pending update) */
static void commit_destroy(struct msm_commit *c)
{
	struct msm_drm_private *priv = c->dev->dev_private;
	uint32_t crtc_mask = c->crtc_mask;
	uint32_t plane_mask = c->plane_mask;

	/* End_atomic */
	spin_lock(&priv->pending_crtcs_event.lock);
	DBG("end: %08x", crtc_mask);
	priv->pending_crtcs &= ~crtc_mask;
	priv->pending_planes &= ~plane_mask;
	wake_up_all_locked(&priv->pending_crtcs_event);
	spin_unlock(&priv->pending_crtcs_event.lock);

	if (c->nonblock)
		kfree(c);
}

static void msm_atomic_wait_for_commit_done(
		struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct msm_drm_private *priv = old_state->dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int i;

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->active)
			continue;

		kms->funcs->wait_for_crtc_commit_done(kms, crtc);
	}
}

static void
msm_disable_outputs(struct drm_device *dev, struct drm_atomic_state *old_state)
{
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i;

	SDE_ATRACE_BEGIN("msm_disable");
	for_each_old_connector_in_state(old_state, connector,
			old_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_crtc_state *old_crtc_state;

		/*
		 * Shut down everything that's in the changeset and currently
		 * still on. So need to check the old, saved state.
		 */
		if (!old_conn_state->crtc)
			continue;

		old_crtc_state = drm_atomic_get_old_crtc_state(old_state,
							old_conn_state->crtc);

		if (!old_crtc_state->active ||
		    !drm_atomic_crtc_needs_modeset(old_conn_state->crtc->state))
			continue;

		encoder = old_conn_state->best_encoder;

		/* We shouldn't get this far if we didn't previously have
		 * an encoder.. but WARN_ON() rather than explode.
		 */
		if (WARN_ON(!encoder))
			continue;

		if (_msm_seamless_for_conn(connector, old_conn_state, false))
			continue;

		funcs = encoder->helper_private;

		DRM_DEBUG_ATOMIC("disabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call disable hooks twice.
		 */
		drm_bridge_disable(encoder->bridge);

		/* Right function depends upon target state. */
		if (connector->state->crtc && funcs->prepare)
			funcs->prepare(encoder);
		else if (funcs->disable)
			funcs->disable(encoder);
		else
			funcs->dpms(encoder, DRM_MODE_DPMS_OFF);

		drm_bridge_post_disable(encoder->bridge);
	}

	for_each_old_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		/* Shut down everything that needs a full modeset. */
		if (!drm_atomic_crtc_needs_modeset(crtc->state))
			continue;

		if (!old_crtc_state->active)
			continue;

		if (_msm_seamless_for_crtc(dev, old_state, crtc->state, false))
			continue;

		funcs = crtc->helper_private;

		DRM_DEBUG_ATOMIC("disabling [CRTC:%d]\n",
				 crtc->base.id);

		/* Right function depends upon target state. */
		if (crtc->state->enable && funcs->prepare)
			funcs->prepare(crtc);
		else if (funcs->disable)
			funcs->disable(crtc);
		else
			funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
	}
	SDE_ATRACE_END("msm_disable");
}

static void
msm_crtc_set_mode(struct drm_device *dev, struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	int i;

	for_each_old_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		if (!crtc->state->mode_changed)
			continue;

		funcs = crtc->helper_private;

		if (crtc->state->enable && funcs->mode_set_nofb) {
			DRM_DEBUG_ATOMIC("modeset on [CRTC:%d]\n",
					 crtc->base.id);

			funcs->mode_set_nofb(crtc);
		}
	}

	for_each_old_connector_in_state(old_state, connector,
			old_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_crtc_state *new_crtc_state;
		struct drm_encoder *encoder;
		struct drm_display_mode *mode, *adjusted_mode;

		if (!connector->state->best_encoder)
			continue;

		encoder = connector->state->best_encoder;
		funcs = encoder->helper_private;
		new_crtc_state = connector->state->crtc->state;
		mode = &new_crtc_state->mode;
		adjusted_mode = &new_crtc_state->adjusted_mode;

		if (!new_crtc_state->mode_changed &&
				new_crtc_state->connectors_changed) {
			if (_msm_seamless_for_conn(connector,
					old_conn_state, false))
				continue;
		} else if (!new_crtc_state->mode_changed) {
			continue;
		}

		DRM_DEBUG_ATOMIC("modeset on [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		SDE_ATRACE_BEGIN("msm_set_mode");
		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call mode_set hooks twice.
		 */
		if (funcs->mode_set)
			funcs->mode_set(encoder, mode, adjusted_mode);

		drm_bridge_mode_set(encoder->bridge, mode, adjusted_mode);
		SDE_ATRACE_END("msm_set_mode");
	}
}

/**
 * msm_atomic_helper_commit_modeset_disables - modeset commit to disable outputs
 * @dev: DRM device
 * @old_state: atomic state object with old state structures
 *
 * This function shuts down all the outputs that need to be shut down and
 * prepares them (if required) with the new mode.
 *
 * For compatibility with legacy crtc helpers this should be called before
 * drm_atomic_helper_commit_planes(), which is what the default commit function
 * does. But drivers with different needs can group the modeset commits together
 * and do the plane commits at the end. This is useful for drivers doing runtime
 * PM since planes updates then only happen when the CRTC is actually enabled.
 */
void msm_atomic_helper_commit_modeset_disables(struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	msm_disable_outputs(dev, old_state);

	drm_atomic_helper_update_legacy_modeset_state(dev, old_state);

	msm_crtc_set_mode(dev, old_state);
}

/**
 * msm_atomic_helper_commit_modeset_enables - modeset commit to enable outputs
 * @dev: DRM device
 * @old_state: atomic state object with old state structures
 *
 * This function enables all the outputs with the new configuration which had to
 * be turned off for the update.
 *
 * For compatibility with legacy crtc helpers this should be called after
 * drm_atomic_helper_commit_planes(), which is what the default commit function
 * does. But drivers with different needs can group the modeset commits together
 * and do the plane commits at the end. This is useful for drivers doing runtime
 * PM since planes updates then only happen when the CRTC is actually enabled.
 */
static void msm_atomic_helper_commit_modeset_enables(struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct drm_crtc_state *new_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *new_conn_state;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int bridge_enable_count = 0;
	int i;

	SDE_ATRACE_BEGIN("msm_enable");
	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		/* Need to filter out CRTCs where only planes change. */
		if (!drm_atomic_crtc_needs_modeset(new_crtc_state))
			continue;

		if (!new_crtc_state->active)
			continue;

		if (_msm_seamless_for_crtc(dev, old_state, crtc->state, true))
			continue;

		funcs = crtc->helper_private;

		if (crtc->state->enable) {
			DRM_DEBUG_ATOMIC("enabling [CRTC:%d]\n",
					 crtc->base.id);

			if (funcs->atomic_enable)
				funcs->atomic_enable(crtc, old_crtc_state);
			else
				funcs->commit(crtc);
		}

		if (msm_needs_vblank_pre_modeset(
					&new_crtc_state->adjusted_mode))
			drm_crtc_wait_one_vblank(crtc);

	}

	for_each_new_connector_in_state(old_state, connector,
			new_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_connector_state *old_conn_state;

		if (!new_conn_state->best_encoder)
			continue;

		if (!new_conn_state->crtc->state->active ||
				!drm_atomic_crtc_needs_modeset(
					new_conn_state->crtc->state))
			continue;

		old_conn_state = drm_atomic_get_old_connector_state(
				old_state, connector);
		if (_msm_seamless_for_conn(connector, old_conn_state, true))
			continue;

		encoder = connector->state->best_encoder;
		funcs = encoder->helper_private;

		DRM_DEBUG_ATOMIC("enabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call enable hooks twice.
		 */
		drm_bridge_pre_enable(encoder->bridge);
		++bridge_enable_count;

		if (funcs->enable)
			funcs->enable(encoder);
		else
			funcs->commit(encoder);
	}

	if (kms && kms->funcs && kms->funcs->commit) {
		DRM_DEBUG_ATOMIC("triggering commit\n");
		kms->funcs->commit(kms, old_state);
	}

	/* If no bridges were pre_enabled, skip iterating over them again */
	if (bridge_enable_count == 0) {
		SDE_ATRACE_END("msm_enable");
		return;
	}

	for_each_new_connector_in_state(old_state, connector,
			new_conn_state, i) {
		struct drm_encoder *encoder;
		struct drm_connector_state *old_conn_state;

		if (!new_conn_state->best_encoder)
			continue;

		if (!new_conn_state->crtc->state->active ||
		    !drm_atomic_crtc_needs_modeset(
				    new_conn_state->crtc->state))
			continue;

		old_conn_state = drm_atomic_get_old_connector_state(
				old_state, connector);
		if (_msm_seamless_for_conn(connector, old_conn_state, true))
			continue;

		encoder = connector->state->best_encoder;

		DRM_DEBUG_ATOMIC("bridge enable enabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		drm_bridge_enable(encoder->bridge);
	}
	SDE_ATRACE_END("msm_enable");
}

int msm_atomic_prepare_fb(struct drm_plane *plane,
			  struct drm_plane_state *new_state)
{
	struct msm_drm_private *priv = plane->dev->dev_private;
	struct msm_kms *kms = priv->kms;
	struct drm_gem_object *obj;
	struct msm_gem_object *msm_obj;
	struct dma_fence *fence;

	if (!new_state->fb)
		return 0;

	obj = msm_framebuffer_bo(new_state->fb, 0);
	msm_obj = to_msm_bo(obj);
	fence = reservation_object_get_excl_rcu(msm_obj->resv);

	drm_atomic_set_fence_for_plane(new_state, fence);

	return msm_framebuffer_prepare(new_state->fb, kms->aspace);
}

/* The (potentially) asynchronous part of the commit.  At this point
 * nothing can fail short of armageddon.
 */
static void complete_commit(struct msm_commit *c)
{
	struct drm_atomic_state *state = c->state;
	struct drm_device *dev = state->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	drm_atomic_helper_wait_for_fences(dev, state, false);

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00005 */
	if ((priv != NULL) && (priv->upper_unit_is_connected == DRM_UPPER_UNIT_IS_NOT_CONNECTED)) {
		int i;
		struct drm_crtc *crtc;
		struct drm_crtc_state *old_crtc_state;

		pr_debug("%s: upper unit is not connected\n", __func__);

		for_each_old_crtc_in_state(state, crtc, old_crtc_state, i) {
			pr_debug("%s:crtc->index=%d,%d\n", __func__, crtc->index, crtc->base.id);
			if (drm_crtc_index(crtc) == 0) {
				goto exit;
			}
		}
	}
#endif /* CONFIG_SHARP_DISPLAY */

	kms->funcs->prepare_commit(kms, state);

	msm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
				DRM_PLANE_COMMIT_ACTIVE_ONLY);

	msm_atomic_helper_commit_modeset_enables(dev, state);

	/* NOTE: _wait_for_vblanks() only waits for vblank on
	 * enabled CRTCs.  So we end up faulting when disabling
	 * due to (potentially) unref'ing the outgoing fb's
	 * before the vblank when the disable has latched.
	 *
	 * But if it did wait on disabled (or newly disabled)
	 * CRTCs, that would be racy (ie. we could have missed
	 * the irq.  We need some way to poll for pipe shut
	 * down.  Or just live with occasionally hitting the
	 * timeout in the CRTC disable path (which really should
	 * not be critical path)
	 */

	msm_atomic_wait_for_commit_done(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);

	kms->funcs->complete_commit(kms, state);

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00005 */
exit:
#endif /* CONFIG_SHARP_DISPLAY */

	drm_atomic_state_put(state);

	commit_destroy(c);
}

static void _msm_drm_commit_work_cb(struct kthread_work *work)
{
	struct msm_commit *commit = NULL;

	if (!work) {
		DRM_ERROR("%s: Invalid commit work data!\n", __func__);
		return;
	}

	commit = container_of(work, struct msm_commit, commit_work);

	SDE_ATRACE_BEGIN("complete_commit");
	complete_commit(commit);
	SDE_ATRACE_END("complete_commit");
}

static struct msm_commit *commit_init(struct drm_atomic_state *state,
	bool nonblock)
{
	struct msm_commit *c = kzalloc(sizeof(*c), GFP_KERNEL);

	if (!c)
		return NULL;

	c->dev = state->dev;
	c->state = state;
	c->nonblock = nonblock;

	kthread_init_work(&c->commit_work, _msm_drm_commit_work_cb);

	return c;
}

/* Start display thread function */
static void msm_atomic_commit_dispatch(struct drm_device *dev,
		struct drm_atomic_state *state, struct msm_commit *commit)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc = NULL;
	struct drm_crtc_state *crtc_state = NULL;
	int ret = -ECANCELED, i = 0, j = 0;
	bool nonblock;

	/* cache since work will kfree commit in non-blocking case */
	nonblock = commit->nonblock;

	for_each_old_crtc_in_state(state, crtc, crtc_state, i) {
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
		if (drm_crtc_index(crtc) == 0) {
			msm_atomic_boost_commit_pending(priv);
		}
#endif /* CONFIG_SHARP_DISPLAY */
		for (j = 0; j < priv->num_crtcs; j++) {
			if (priv->disp_thread[j].crtc_id ==
						crtc->base.id) {
				if (priv->disp_thread[j].thread) {
					kthread_queue_work(
						&priv->disp_thread[j].worker,
							&commit->commit_work);
					/* only return zero if work is
					 * queued successfully.
					 */
					ret = 0;
				} else {
					DRM_ERROR(" Error for crtc_id: %d\n",
						priv->disp_thread[j].crtc_id);
					ret = -EINVAL;
				}
				break;
			}
		}
		/*
		 * TODO: handle cases where there will be more than
		 * one crtc per commit cycle. Remove this check then.
		 * Current assumption is there will be only one crtc
		 * per commit cycle.
		 */
		if (j < priv->num_crtcs)
			break;
	}

	if (ret) {
		if (ret == -EINVAL)
			DRM_ERROR("failed to dispatch commit to any CRTC\n");
		else
			DRM_DEBUG_DRIVER_RATELIMITED("empty crtc state\n");

		/**
		 * this is not expected to happen, but at this point the state
		 * has been swapped, but we couldn't dispatch to a crtc thread.
		 * fallback now to a synchronous complete_commit to try and
		 * ensure that SW and HW state don't get out of sync.
		 */
		complete_commit(commit);
	} else if (!nonblock) {
		kthread_flush_work(&commit->commit_work);
	}

	/* free nonblocking commits in this context, after processing */
	if (!nonblock)
		kfree(commit);
}

/**
 * drm_atomic_helper_commit - commit validated state object
 * @dev: DRM device
 * @state: the driver state object
 * @nonblock: nonblocking commit
 *
 * This function commits a with drm_atomic_helper_check() pre-validated state
 * object. This can still fail when e.g. the framebuffer reservation fails.
 *
 * RETURNS
 * Zero for success or -errno.
 */
int msm_atomic_commit(struct drm_device *dev,
		struct drm_atomic_state *state, bool nonblock)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_commit *c;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	int i, ret;

	if (!priv || priv->shutdown_in_progress) {
		DRM_ERROR("priv is null or shutdwon is in-progress\n");
		return -EINVAL;
	}

	SDE_ATRACE_BEGIN("atomic_commit");
	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret) {
		SDE_ATRACE_END("atomic_commit");
		return ret;
	}

	c = commit_init(state, nonblock);
	if (!c) {
		ret = -ENOMEM;
		goto error;
	}

	/*
	 * Figure out what crtcs we have:
	 */
	for_each_new_crtc_in_state(state, crtc, crtc_state, i)
		c->crtc_mask |= drm_crtc_mask(crtc);

	/*
	 * Figure out what fence to wait for:
	 */
	for_each_oldnew_plane_in_state(state, plane, old_plane_state,
			new_plane_state, i) {
		if ((new_plane_state->fb != old_plane_state->fb)
				&& new_plane_state->fb) {
			struct drm_gem_object *obj =
				msm_framebuffer_bo(new_plane_state->fb, 0);
			struct msm_gem_object *msm_obj = to_msm_bo(obj);
			struct dma_fence *fence =
				reservation_object_get_excl_rcu(msm_obj->resv);

			drm_atomic_set_fence_for_plane(new_plane_state, fence);
		}
		c->plane_mask |= (1 << drm_plane_index(plane));
	}

	/* Protection for prepare_fence callback */
retry:
	ret = drm_modeset_lock(&state->dev->mode_config.connection_mutex,
		state->acquire_ctx);

	if (ret == -EDEADLK) {
		drm_modeset_backoff(state->acquire_ctx);
		goto retry;
	}

	/*
	 * Wait for pending updates on any of the same crtc's and then
	 * mark our set of crtc's as busy:
	 */

	/* Start Atomic */
	spin_lock(&priv->pending_crtcs_event.lock);
	ret = wait_event_interruptible_locked(priv->pending_crtcs_event,
			!(priv->pending_crtcs & c->crtc_mask) &&
			!(priv->pending_planes & c->plane_mask));
	if (ret == 0) {
		DBG("start: %08x", c->crtc_mask);
		priv->pending_crtcs |= c->crtc_mask;
		priv->pending_planes |= c->plane_mask;
	}
	spin_unlock(&priv->pending_crtcs_event.lock);

	if (ret)
		goto err_free;

	WARN_ON(drm_atomic_helper_swap_state(state, false) < 0);

	/*
	 * Provide the driver a chance to prepare for output fences. This is
	 * done after the point of no return, but before asynchronous commits
	 * are dispatched to work queues, so that the fence preparation is
	 * finished before the .atomic_commit returns.
	 */
	if (priv && priv->kms && priv->kms->funcs &&
			priv->kms->funcs->prepare_fence)
		priv->kms->funcs->prepare_fence(priv->kms, state);

	/*
	 * Everything below can be run asynchronously without the need to grab
	 * any modeset locks at all under one conditions: It must be guaranteed
	 * that the asynchronous work has either been cancelled (if the driver
	 * supports it, which at least requires that the framebuffers get
	 * cleaned up with drm_atomic_helper_cleanup_planes()) or completed
	 * before the new state gets committed on the software side with
	 * drm_atomic_helper_swap_state().
	 *
	 * This scheme allows new atomic state updates to be prepared and
	 * checked in parallel to the asynchronous completion of the previous
	 * update. Which is important since compositors need to figure out the
	 * composition of the next frame right after having submitted the
	 * current layout
	 */

	drm_atomic_state_get(state);
	msm_atomic_commit_dispatch(dev, state, c);

	SDE_ATRACE_END("atomic_commit");

	return 0;
err_free:
	kfree(c);
error:
	drm_atomic_helper_cleanup_planes(dev, state);
	SDE_ATRACE_END("atomic_commit");
	return ret;
}

struct drm_atomic_state *msm_atomic_state_alloc(struct drm_device *dev)
{
	struct msm_kms_state *state = kzalloc(sizeof(*state), GFP_KERNEL);

	if (!state || drm_atomic_state_init(dev, &state->base) < 0) {
		kfree(state);
		return NULL;
	}

	return &state->base;
}

void msm_atomic_state_clear(struct drm_atomic_state *s)
{
	struct msm_kms_state *state = to_kms_state(s);

	drm_atomic_state_default_clear(&state->base);
	kfree(state->state);
	state->state = NULL;
}

void msm_atomic_state_free(struct drm_atomic_state *state)
{
	kfree(to_kms_state(state)->state);
	drm_atomic_state_default_release(state);
	kfree(state);
}

void msm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	kms->funcs->prepare_commit(kms, state);

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_planes(dev, state, 0);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	msm_atomic_wait_for_commit_done(dev, state);

	kms->funcs->complete_commit(kms, state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00028 */
static int msm_atomic_update_panel_timing(struct dsi_display *display,
	struct mdp_mipi_clkchg_param *clkchg_param)
{
	int rc = 0;
	int cmd_cnt;
	struct dsi_cmd_desc *cmds;

	unsigned char addr_value[][52] = {
		{0xDE, 0x00},	// Switch to Page0
		{0xC4, 0x03,
		       0x06,	// SLT
		       0x30,
		       0x54,	// SLT
		       0x00,	// VFP
		       0x09,	// VFP
		       0x00,	// VBP
		       0x0C,	// VBP
		       0x0C},
		{0xC6, 0x00,
		       0x12,	// SLT_2C
		       0x45,	// SLT_2C
		       0x00,	// VFP_2C
		       0x07,	// VFP_2C
		       0x00,	// VBP_2C
		       0x0C,	// VBP_2C
		       0x01,	// CKV0_ON_2C
		       0x20,	// CKV0_OFF_2C
		       0x25,
		       0x30,
		       0x01,
		       0x49,
		       0x01,
		       0x49,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x00,
		       0x03,
		       0x00,
		       0x00,	// RGB_EQ1_2C
		       0x00,	// RGB_EQ2_2C
		       0x45,	// RGB_EQ3_2C
		       0x01,	// RGB_CHGEN_ON_2C
		       0x45,	// RGB_CHGEN_OFF_2C
		       0x4B,
		       0x02,
		       0x4B,
		       0x05,
		       0x05,
		       0x05,
		       0x05},
		{0xCE, 0x00,
		       0x41,	// STV_ON_2C
		       0x25,	// STV_OFF_2C
		       0x01,
		       0x40,
		       0x03,
		       0x49,
		       0x00,
		       0x99,
		       0x01,
		       0x49,
		       0x01,
		       0x49},
		{0xDE, 0x02},	// Switch to Page2
		{0xC7, 0x08},	// OSCD_ADJ
		{0xDE, 0x00},	// Switch to Page0
	};
	struct dsi_cmd_desc paneltiming_cmd[] = {
		{{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, MIPI_DSI_MSG_USE_LPM,
			0, 0, 2, addr_value[0], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_LONG_WRITE, MIPI_DSI_MSG_USE_LPM,
			0, 0,10, addr_value[1], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_LONG_WRITE, MIPI_DSI_MSG_USE_LPM,
			0, 0,52, addr_value[2], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_LONG_WRITE, MIPI_DSI_MSG_USE_LPM,
			0, 0,14, addr_value[3], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, MIPI_DSI_MSG_USE_LPM,
			0, 0, 2, addr_value[4], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, MIPI_DSI_MSG_USE_LPM,
			0, 0, 2, addr_value[5], 0, 0}, 0, 0},
		{{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, MIPI_DSI_MSG_USE_LPM,
			0, 0, 2, addr_value[6], 0, 0}, 1, 0},
	};

	pr_debug("%s\n", __func__);

	addr_value[1][7] = clkchg_param->panel.vbp[0];
	addr_value[1][8] = clkchg_param->panel.vbp[1];
	addr_value[1][5] = clkchg_param->panel.vfp[0];
	addr_value[1][6] = clkchg_param->panel.vfp[1];
	addr_value[1][2] = clkchg_param->panel.slt[0];
	addr_value[1][4] = clkchg_param->panel.slt[1];

	addr_value[2][6] = clkchg_param->panel.vbp_2c[0];
	addr_value[2][7] = clkchg_param->panel.vbp_2c[1];
	addr_value[2][4] = clkchg_param->panel.vfp_2c[0];
	addr_value[2][5] = clkchg_param->panel.vfp_2c[1];
	addr_value[2][2] = clkchg_param->panel.slt_2c[0];
	addr_value[2][3] = clkchg_param->panel.slt_2c[1];

	addr_value[3][2] = clkchg_param->panel.stv_on_2c;
	addr_value[3][3] = clkchg_param->panel.stv_off_2c;

	addr_value[2][8] = clkchg_param->panel.ckv0_on_2c;
	addr_value[2][9] = clkchg_param->panel.ckv0_off_2c;
	addr_value[2][43] = clkchg_param->panel.rgb_chgen_on_2c;
	addr_value[2][44] = clkchg_param->panel.rgb_chgen_off_2c;
	addr_value[2][40] = clkchg_param->panel.rgb_eq1_2c;
	addr_value[2][41] = clkchg_param->panel.rgb_eq2_2c;
	addr_value[2][42] = clkchg_param->panel.rgb_eq3_2c;

	addr_value[5][1] = clkchg_param->panel.oscd_adj;

	cmd_cnt = ARRAY_SIZE(paneltiming_cmd);
	cmds = paneltiming_cmd;

	rc = drm_cmn_panel_cmds_transfer(display, cmds, cmd_cnt);
	if (rc) {
		pr_err("%s:failed to set cmds, rc=%d\n", __func__, rc);
	}

	return rc;
}

static void msm_atomic_set_phys_cached_mode(struct dsi_display *display,
	struct dsi_display_mode *adj_mode)
{
	struct drm_display_mode drm_mode;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	struct sde_encoder_phys *phys_enc = NULL;
	int ctrl_count;
	int i = 0;

	pr_debug("%s\n", __func__);

	memset(&drm_mode, 0x00, sizeof(struct drm_display_mode));
	ctrl_count = display->ctrl_count;

	drm_mode.hdisplay = adj_mode->timing.h_active * ctrl_count;
	drm_mode.hsync_start = drm_mode.hdisplay +
				adj_mode->timing.h_front_porch * ctrl_count;
	drm_mode.hsync_end = drm_mode.hsync_start +
				adj_mode->timing.h_sync_width * ctrl_count;
	drm_mode.htotal = drm_mode.hsync_end +
				adj_mode->timing.h_back_porch * ctrl_count;

	drm_mode.vdisplay = adj_mode->timing.v_active;
	drm_mode.vsync_start = drm_mode.vdisplay +
				adj_mode->timing.v_front_porch;
	drm_mode.vsync_end = drm_mode.vsync_start +
				adj_mode->timing.v_sync_width;
	drm_mode.vtotal = drm_mode.vsync_end +
				adj_mode->timing.v_back_porch;

	drm_mode.vrefresh = adj_mode->timing.refresh_rate;
	drm_mode.clock = adj_mode->pixel_clk_khz * ctrl_count;

	drm_mode.private = (int *)adj_mode->priv_info;

	for (i = 0; i < ctrl_count; i++) {
		cmd_enc = get_sde_encoder_phys_cmd(i);
		if (cmd_enc) {
			phys_enc = &cmd_enc->base;

			phys_enc->cached_mode = drm_mode;
		}
	}
}

static void msm_atomic_mipiclk_adjusted_mode(struct dsi_display *display,
	struct dsi_display_mode *adj_mode,
	struct mdp_mipi_clkchg_param *clkchg_param)
{
	struct dsi_display_mode_priv_info *priv_info = NULL;
	int i;

	pr_debug("%s\n", __func__);

	adj_mode->timing.h_active      =
		clkchg_param->host.display_width;
	adj_mode->timing.v_active      =
		clkchg_param->host.display_height;
	adj_mode->timing.h_sync_width  =
		clkchg_param->host.hsync_pulse_width;
	adj_mode->timing.h_back_porch  =
		clkchg_param->host.h_back_porch;
	adj_mode->timing.h_front_porch =
		clkchg_param->host.h_front_porch;
	adj_mode->timing.v_sync_width  =
		clkchg_param->host.vsync_pulse_width;
	adj_mode->timing.v_back_porch  =
		clkchg_param->host.v_back_porch;
	adj_mode->timing.v_front_porch =
		clkchg_param->host.v_front_porch;
	adj_mode->timing.refresh_rate  =
		clkchg_param->host.frame_rate;
	adj_mode->timing.clk_rate_hz   =
		clkchg_param->host.clock_rate;

	if (display->ctrl_count > 1) {
		adj_mode->timing.h_active /= display->ctrl_count;
	}

	adj_mode->pixel_clk_khz = (DSI_H_TOTAL_DSC(&adj_mode->timing) *
			DSI_V_TOTAL(&adj_mode->timing) *
			adj_mode->timing.refresh_rate) / 1000;

	priv_info = (struct dsi_display_mode_priv_info*)adj_mode->priv_info;
	for (i = 0;i < priv_info->phy_timing_len;i++) {
		priv_info->phy_timing_val[i] =
			clkchg_param->host.timing_ctrl[i];
	}
	priv_info->clk_rate_hz = clkchg_param->host.clock_rate;
}

static int msm_atomic_setup_timing_engine(struct drm_crtc *crtc,
	struct dsi_display *display)
{
	struct drm_display_mode *adjusted_mode;
	struct dsi_display_mode *dsi_mode;
	int rc = 0;
	int ctrl_count;

	pr_debug("%s\n", __func__);

	dsi_mode = display->panel->cur_mode;
	adjusted_mode = &crtc->state->adjusted_mode;
	if (!adjusted_mode) {
		pr_err("[%s]adjusted_mode is null\n", __func__);
		return -EINVAL;
	}
	ctrl_count = display->ctrl_count;

	adjusted_mode->hdisplay = dsi_mode->timing.h_active * ctrl_count;
	adjusted_mode->hsync_start = adjusted_mode->hdisplay +
				dsi_mode->timing.h_front_porch * ctrl_count;
	adjusted_mode->hsync_end = adjusted_mode->hsync_start +
				dsi_mode->timing.h_sync_width * ctrl_count;
	adjusted_mode->htotal = adjusted_mode->hsync_end +
				dsi_mode->timing.h_back_porch * ctrl_count;

	adjusted_mode->vdisplay = dsi_mode->timing.v_active;
	adjusted_mode->vsync_start = adjusted_mode->vdisplay +
				dsi_mode->timing.v_front_porch;
	adjusted_mode->vsync_end = adjusted_mode->vsync_start +
				dsi_mode->timing.v_sync_width;
	adjusted_mode->vtotal = adjusted_mode->vsync_end +
				dsi_mode->timing.v_back_porch;

	adjusted_mode->vrefresh = dsi_mode->timing.refresh_rate;
	adjusted_mode->clock = dsi_mode->pixel_clk_khz * ctrl_count;

	adjusted_mode->private = (int *)dsi_mode->priv_info;

	return rc;
}

static int msm_atomic_mipiclk_config_dsi(struct dsi_display *display,
	struct drm_crtc *crtc, struct mdp_mipi_clkchg_param *clkchg_param)
{
	struct dsi_display_mode *adj_mode;
	int rc = 0;

	pr_debug("%s\n", __func__);

	if (!display->panel->cur_mode) {
		pr_err("[%s]failed to display->panel->cur_mode is null\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&display->panel->panel_lock);

	adj_mode = display->panel->cur_mode;

	msm_atomic_mipiclk_adjusted_mode(display, adj_mode, clkchg_param);

	adj_mode->timing.dsc_enabled = display->config.video_timing.dsc_enabled;
	adj_mode->timing.dsc         = display->config.video_timing.dsc;

	msm_atomic_set_phys_cached_mode(display, adj_mode);

	mutex_unlock(&display->panel->panel_lock);

	rc = dsi_display_set_mode_sub_wrap(display, adj_mode, 0x00);
	if (rc) {
		pr_err("[%s]failed to dsi_display_set_mode, rc=%d\n",
			__func__, rc);
	}

	mutex_unlock(&display->display_lock);

	return rc;
}

static void msm_atomic_update_mipiclk_chg_log(
	struct mdp_mipi_clkchg_param *clkchg_param)
{
	int i;

	pr_debug("[%s]param->host.frame_rate        = %10d", __func__  , clkchg_param->host.frame_rate       );
	pr_debug("[%s]param->host.clock_rate        = %10d", __func__  , clkchg_param->host.clock_rate       );
	pr_debug("[%s]param->host.display_width     = %10d", __func__  , clkchg_param->host.display_width    );
	pr_debug("[%s]param->host.display_height    = %10d", __func__  , clkchg_param->host.display_height   );
	pr_debug("[%s]param->host.hsync_pulse_width = %10d", __func__  , clkchg_param->host.hsync_pulse_width);
	pr_debug("[%s]param->host.h_back_porch      = %10d", __func__  , clkchg_param->host.h_back_porch     );
	pr_debug("[%s]param->host.h_front_porch     = %10d", __func__  , clkchg_param->host.h_front_porch    );
	pr_debug("[%s]param->host.vsync_pulse_width = %10d", __func__  , clkchg_param->host.vsync_pulse_width);
	pr_debug("[%s]param->host.v_back_porch      = %10d", __func__  , clkchg_param->host.v_back_porch     );
	pr_debug("[%s]param->host.v_front_porch     = %10d", __func__  , clkchg_param->host.v_front_porch    );
	for (i = 0; i < 14; i++) {
		pr_debug("[%s]param->host.timing_ctrl[%02d]   = 0x%02X", __func__, i, clkchg_param->host.timing_ctrl[i]);
	}

	pr_debug("[%s]param->panel.vbp[0]           = 0x%02X",__func__, clkchg_param->panel.vbp[0]          );
	pr_debug("[%s]param->panel.vbp[1]           = 0x%02X",__func__, clkchg_param->panel.vbp[1]          );
	pr_debug("[%s]param->panel.vfp[0]           = 0x%02X",__func__, clkchg_param->panel.vfp[0]          );
	pr_debug("[%s]param->panel.vfp[1]           = 0x%02X",__func__, clkchg_param->panel.vfp[1]          );
	pr_debug("[%s]param->panel.slt[0]           = 0x%02X",__func__, clkchg_param->panel.slt[0]          );
	pr_debug("[%s]param->panel.slt[1]           = 0x%02X",__func__, clkchg_param->panel.slt[1]          );
	pr_debug("[%s]param->panel.vbp_2c[0]        = 0x%02X",__func__, clkchg_param->panel.vbp_2c[0]       );
	pr_debug("[%s]param->panel.vbp_2c[1]        = 0x%02X",__func__, clkchg_param->panel.vbp_2c[1]       );
	pr_debug("[%s]param->panel.vfp_2c[0]        = 0x%02X",__func__, clkchg_param->panel.vfp_2c[0]       );
	pr_debug("[%s]param->panel.vfp_2c[1]        = 0x%02X",__func__, clkchg_param->panel.vfp_2c[1]       );
	pr_debug("[%s]param->panel.slt_2c[0]        = 0x%02X",__func__, clkchg_param->panel.slt_2c[0]       );
	pr_debug("[%s]param->panel.slt_2c[1]        = 0x%02X",__func__, clkchg_param->panel.slt_2c[1]       );
	pr_debug("[%s]param->panel.stv_on_2c        = 0x%02X",__func__, clkchg_param->panel.stv_on_2c       );
	pr_debug("[%s]param->panel.stv_off_2c       = 0x%02X",__func__, clkchg_param->panel.stv_off_2c      );
	pr_debug("[%s]param->panel.ckv0_on_2c       = 0x%02X",__func__, clkchg_param->panel.ckv0_on_2c      );
	pr_debug("[%s]param->panel.ckv0_off_2c      = 0x%02X",__func__, clkchg_param->panel.ckv0_off_2c     );
	pr_debug("[%s]param->panel.rgb_chgen_on_2c  = 0x%02X",__func__, clkchg_param->panel.rgb_chgen_on_2c );
	pr_debug("[%s]param->panel.rgb_chgen_off_2c = 0x%02X",__func__, clkchg_param->panel.rgb_chgen_off_2c);
	pr_debug("[%s]param->panel.rgb_eq1_2c       = 0x%02X",__func__, clkchg_param->panel.rgb_eq1_2c      );
	pr_debug("[%s]param->panel.rgb_eq2_2c       = 0x%02X",__func__, clkchg_param->panel.rgb_eq2_2c      );
	pr_debug("[%s]param->panel.rgb_eq3_2c       = 0x%02X",__func__, clkchg_param->panel.rgb_eq3_2c      );
	pr_debug("[%s]param->panel.oscd_adj         = 0x%02X",__func__, clkchg_param->panel.oscd_adj        );
}

static int msm_atomic_update_mipiclk_chg(struct drm_crtc *crtc,
	struct mdp_mipi_clkchg_param *clkchg_param)
{
	struct dsi_display *display;
	int rc = 0;
	int clk_state;

	pr_debug("[%s] in\n", __func__);

	display = msm_drm_get_dsi_display();
	if (!display) {
		pr_err("[%s]failed to display is null\n",
			__func__);
		return -EINVAL;
	}

	clk_state = dsi_clk_get_state(display->mdp_clk_handle);

	msm_atomic_update_mipiclk_chg_log(clkchg_param);

	msm_atomic_update_panel_timing(display, clkchg_param);

	if (clk_state) {
		rc = dsi_display_clk_ctrl(display->mdp_clk_handle
						, clk_state, DSI_CLK_OFF);
		if (rc) {
			pr_err("[%s]failed to disable MDP clocks, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}

	rc = msm_atomic_mipiclk_config_dsi(display, crtc, clkchg_param);
	if (rc) {
		pr_err("[%s]failed to update DSI clocks, rc=%d\n",
			__func__, rc);
		return rc;
	}

	rc = msm_atomic_setup_timing_engine(crtc, display);
	if (rc) {
		pr_err("[%s]failed to msm_atomic_setup_timing_engine, rc=%d\n",
			__func__, rc);
		return rc;
	}

	if (clk_state) {
		rc = dsi_display_clk_ctrl(display->mdp_clk_handle
						, clk_state, DSI_CLK_ON);
		if (rc) {
			pr_err("[%s]failed to enable MDP clocks, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}

	pr_debug("[%s] out\n", __func__);

	return rc;
}

int msm_atomic_update_mipiclk_resume(struct dsi_display *display,
	struct dsi_display_mode *adj_mode)
{
	struct msm_drm_private *priv;
	struct mdp_mipi_clkchg_param *clkchg_param = NULL;

	pr_debug("%s\n", __func__);

	priv = display->drm_dev->dev_private;

	if (priv) {
		mutex_lock(&priv->mipiclk_lock);
		clkchg_param = &priv->usr_clkchg_param;

		if (clkchg_param->host.clock_rate) {
			msm_atomic_mipiclk_adjusted_mode(display, adj_mode, clkchg_param);

			msm_atomic_set_phys_cached_mode(display, adj_mode);

			priv->mipiclk_pending = false;
		}
		mutex_unlock(&priv->mipiclk_lock);
	}

	return 0;
}

int msm_atomic_update_mipiclk(struct msm_drm_private *priv, struct drm_crtc *crtc)
{
	struct mdp_mipi_clkchg_param clkchg_param;
	int rc = 0;

	if (!priv) {
		pr_err("[%s]failed to priv is null\n",
			__func__);
		return -EINVAL;
	}

	if (!crtc) {
		pr_err("[%s]failed to crtc is null\n",
			__func__);
		return -EINVAL;
	}

	if (priv->mipiclk_pending) {
		pr_debug("[%s]crtc_state->active=%d\n", __func__, crtc->state->active);
		if (drm_crtc_index(crtc) == 0 && crtc->state->active) {
			mutex_lock(&priv->mipiclk_lock);
			memcpy(&clkchg_param, &priv->usr_clkchg_param,
					sizeof(clkchg_param));
			rc = msm_atomic_update_mipiclk_chg(crtc, &clkchg_param);
			priv->mipiclk_pending = false;
			mutex_unlock(&priv->mipiclk_lock);
		}
	}

	return rc;
}

int msm_atomic_update_panel_timing_resume(struct dsi_display *display)
{
	int rc = 0;
	int clk_rate_hz, default_clk_rate_hz;
	struct msm_drm_private *priv = NULL;

	pr_debug("%s\n", __func__);

	if (!display) {
		pr_err("[%s]Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	priv = display->drm_dev->dev_private;
	if (!priv) {
		pr_err("[%s]Invalid dev_private\n", __func__);
		return -EINVAL;
	}

	default_clk_rate_hz = drm_cmn_get_default_clk_rate_hz();
	clk_rate_hz = priv->usr_clkchg_param.host.clock_rate;

	pr_debug("[%s]default_clk_rate_hz = %d, clk_rate_hz = %d\n",
			__func__, default_clk_rate_hz, clk_rate_hz);

	if (default_clk_rate_hz && clk_rate_hz) {
		if (default_clk_rate_hz != clk_rate_hz) {
			priv = display->drm_dev->dev_private;
			rc = msm_atomic_update_panel_timing(display,
				&priv->usr_clkchg_param);
		}
	}

	return rc;
}
#endif /* CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00054 */
void msm_atomic_notify_mode_dpms(struct drm_connector *connector,
	int power_mode)
{
	struct drm_panel_notifier notifier_data;
	int blank;

	switch (power_mode) {
	case SDE_MODE_DPMS_ON:
		blank = DRM_PANEL_EVENT_DPMS_ON;
		break;
	case SDE_MODE_DPMS_LP1:
		blank = DRM_PANEL_EVENT_DPMS_LP1;
		break;
	case SDE_MODE_DPMS_LP2:
		blank = DRM_PANEL_EVENT_DPMS_LP2;
		break;
	case SDE_MODE_DPMS_OFF:
		blank = DRM_PANEL_EVENT_DPMS_OFF;
		break;
	default:
		return;
	}
	pr_debug("%s:power_mode=%d,blank=%d\n", __func__, power_mode, blank);
	notifier_data.data = &blank;
	drm_panel_notifier_call_chain(connector->panel, DRM_PANEL_EVENT_DPMS,
					    &notifier_data);
}
#endif /* CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
void msm_atomic_boost_commit_pending(struct msm_drm_private *priv)
{
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;
	struct dsi_display *display = NULL;
	struct dsi_backlight_config *bl = NULL;

	if (priv != NULL) {
		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
			    (sde_kms->dsi_displays != NULL)) {
				display = sde_kms->dsi_displays[DSI_PRIMARY];
				if ((display != NULL) && (display->panel != NULL)) {
					mutex_lock(&display->panel->curr_boost_lock);
					bl = &display->panel->bl_config;
					if (bl->boost_commit_pending == true) {
						bl->boost_kickoff_pending = true;
						bl->boost_commit_pending = false;
						pr_debug("%s:boost_kickoff_pending is true\n", __func__);
					}
					mutex_unlock(&display->panel->curr_boost_lock);
				} else {
					pr_err("[%s]failed to display is null\n", __func__);
				}
			} else {
				pr_err("[%s]failed to dsi_displays count=%d addr=%p\n",
					__func__, sde_kms->dsi_display_count,
					sde_kms->dsi_displays);
			}
		} else {
			pr_err("[%s]failed to kms is null\n", __func__);
		}
	} else {
		pr_err("[%s]failed to priv is null\n", __func__);
	}
}
#endif /* CONFIG_SHARP_DISPLAY */
