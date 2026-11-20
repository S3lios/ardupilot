#include "mode.h"  
#include "Plane.h"  
  
#if MODE_SMARTRTL_ENABLED  
  
bool ModeSmartRTL::_enter()  
{  
    if (!plane.g2.smart_rtl.enabled() || !plane.g2.smart_rtl.is_active()) {
        return false;  
    }  
    plane.prev_WP_loc = plane.current_loc;  
    plane.next_WP_loc = plane.current_loc;   // tenir la position pendant le cleanup  
    dest_NED_backup.zero();  
    

    set_submode(SubMode::WAIT_FOR_PATH_CLEANUP);  
    return true;  
}  
  
void ModeSmartRTL::_exit()  
{  
    if (_state == SubMode::PATH_FOLLOW && !dest_NED_backup.is_zero()) {  
        if (!plane.g2.smart_rtl.add_point(dest_NED_backup)) {  
            gcs().send_text(MAV_SEVERITY_ERROR, "SmartRTL: lost one point");  
        }  
    }  
    dest_NED_backup.zero();  
    plane.g2.smart_rtl.cancel_request_for_thorough_cleanup();  
}  
  
void ModeSmartRTL::set_submode(SubMode submode)  
{  
    _state = submode;  
    if (submode == SubMode::PATH_FOLLOW) {  
        path_follow_last_pop_fail_ms = 0;  
    }  
}  
  
void ModeSmartRTL::update()  
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    bool alt_threshold_reached = false;
    if (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)) {
        // Climb to RTL_ALTITUDE before turning. This overrides RTL_CLIMB_MIN.
        alt_threshold_reached = plane.current_loc.alt > plane.next_WP_loc.alt;
    } else if (plane.g2.rtl_climb_min > 0) {
        /*
           when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
           until we have climbed by RTL_CLIMB_MIN meters
           */
        alt_threshold_reached = (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min;
    } else {
        return;
    }

    if (!plane.rtl.done_climb && alt_threshold_reached) {
        plane.prev_WP_loc = plane.current_loc;
        plane.setup_alt_slope();
        plane.rtl.done_climb = true;
    }
    if (!plane.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if something goes wrong the plane will
        // eventually turn back and go to RTL instead of going perfectly straight. This also leaves
        // some leeway for fighting wind.
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }
}  
  
void ModeSmartRTL::navigate()  
{  

    const Location rtl_target = plane.calc_best_rally_or_home_location(
        plane.current_loc, plane.home.alt + plane.get_RTL_altitude_cm());

    const float switch_dist_m = (float)abs(plane.aparm.loiter_radius) * 2.0f;

    if (plane.current_loc.get_distance(rtl_target) <= switch_dist_m) {
        dest_NED_backup.zero();
        if (!plane.set_mode(plane.mode_rtl, ModeReason::SMART_RTL_COMPLETE)) {
            set_submode(SubMode::LOITER_OR_LAND);
        }
        return;
    }

    switch (_state) {  
    case SubMode::WAIT_FOR_PATH_CLEANUP:  
        wait_cleanup_run();  
        break;  
    case SubMode::PATH_FOLLOW:  
        path_follow_run();  
        break;  
    case SubMode::LOITER_OR_LAND:  
        uint16_t radius = abs(plane.g.rtl_radius);
        if (radius > 0) {
            plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
        }
        plane.update_loiter(radius); // ou do_RTL + loiter standard  
        break;  
    }  
}  
  
void ModeSmartRTL::wait_cleanup_run()  
{  
    if (plane.g2.smart_rtl.request_thorough_cleanup()) {  
        set_submode(SubMode::PATH_FOLLOW);  
    }  
}  
  
void ModeSmartRTL::path_follow_run()  
{  
    if (plane.auto_state.wp_distance <= plane.get_wp_radius()) {  
        dest_NED_backup.zero();  
  
        Vector3p dest_NED;  
        if (plane.g2.smart_rtl.pop_point(dest_NED)) {  
            dest_NED_backup = dest_NED;  
            path_follow_last_pop_fail_ms = 0;  
  
            Location dest;
            if (!AP::ahrs().get_origin(dest)) {
                IGNORE_RETURN(plane.set_mode(plane.mode_rtl, ModeReason::SMART_RTL_POP_FAILED));
                return;
            }
            dest.offset(dest_NED);  
            dest.set_alt_cm(plane.home.alt + fmax((int32_t)(-dest_NED.z * 100), (int32_t)plane.mode_takeoff.target_alt),  
                            Location::AltFrame::ABSOLUTE);  
  
            plane.prev_WP_loc = plane.next_WP_loc;  
            plane.next_WP_loc = dest;  
            plane.set_next_WP(dest);  
  
            if (plane.g2.smart_rtl.get_num_points() == 0) {  
                set_submode(SubMode::LOITER_OR_LAND);  
            }  
        } else if (path_follow_last_pop_fail_ms == 0) {  
            path_follow_last_pop_fail_ms = AP_HAL::millis();  
        } else if (AP_HAL::millis() - path_follow_last_pop_fail_ms > 10000) {  
            // we failed to pop a point off for 10 seconds.  This is
            // almost certainly a bug. Fall back to rtl mode to avoid getting stuck in smart_rtl mode.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _state = SubMode::LOITER_OR_LAND;
            IGNORE_RETURN(plane.set_mode(plane.mode_rtl, ModeReason::SMART_RTL_POP_FAILED));  
        }
    }

    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}  
  
void ModeSmartRTL::save_position()  
{      

    const bool should_save = plane.arming.is_armed() &&  
        (plane.control_mode->mode_number() != Number::SMART_RTL);  
    

    if (!plane.have_position) {
        return;
    }

    if (plane.current_loc.lat == 0 && plane.current_loc.lng == 0) {
        return;
    }
    plane.g2.smart_rtl.update(true, should_save);  
}  
  
#endif // MODE_SMARTRTL_ENABLED