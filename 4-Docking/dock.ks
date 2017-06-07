@lazyglobal off.

//=============================================================Vector stuff
clearvecdraws().

declare local function init_vectors {
	if target:name:contains("dockingPort") {
		global target_port is target.
	} else {
		global target_port is target:dockingports[0].
	}	
	global roll_mod_1 is 0. //were both 90
	global xaxis is V(0,0,0). 
	global yaxis is V(0,0,0).
	global zaxis is V(0,0,0).
	lock xaxis to -1 * target_port:portfacing:vector.
	//lock yaxis to target_port:ship:facing:vector.
	//lock yaxis to (target_port:portfacing + r(roll_mod_1,0,0)):vector.
	lock yaxis to (target_port:portfacing + r(90,0,0)):vector.
	lock zaxis to vcrs(xaxis, yaxis).
	//lock zaxis to (target_port:portfacing + r(0,-90,0)):vector.
	
	global xaxis_draw is vecdraw(V(0,0,0), xaxis, RGB(1,0,0),"xa",1,true,0.2).
	global yaxis_draw is vecdraw(V(0,0,0), yaxis, RGB(0,1,0),"ya",1,true,0.2).
	global zaxis_draw is vecdraw(V(0,0,0), zaxis, RGB(0,0,1),"za",1,true,0.2).
	global velocity_draw is vecdraw(V(0,0,0), V(0,0,0), RGB(0,1,1),"vel",1,false,0.2).
	global acc_draw is vecdraw(V(0,0,0),V(0,0,0), RGB(1,0,1),"acc",1,false,0.2).
	global y_pos_draw is vecdraw(V(0,0,0), V(0,0,0), RGB(1,0,1),"y_pos",1,true,0.2).
	global z_pos_draw is vecdraw(V(0,0,0), V(0,0,0), RGB(1,1,0),"z_pos",1,true,0.2).
}

declare local function show_vectors {
	local scalar is 10.
	set xaxis_draw:vec to xaxis * scalar.
	set yaxis_draw:vec to yaxis * scalar.
	set zaxis_draw:vec to zaxis * scalar.
	set y_pos_draw:vec to yaxis * vdot(yaxis, target_port:position) * 1.
	set z_pos_draw:vec to zaxis * vdot(zaxis, target_port:position) * 1.
	set velocity_draw:vec to get_velocity() * 50.
}

declare local function get_pos {
	local pos is target_port:position.
	local x_pos is vdot(pos, xaxis).
	local y_pos is vdot(pos, yaxis).
	local z_pos is vdot(pos, zaxis).
	return v(x_pos, y_pos, z_pos).
}

declare local function get_velocity {
	local vel is ship:velocity:obt - target_port:ship:velocity:obt.
	local x_vel is vdot(vel, xaxis).
	local y_vel is vdot(vel, yaxis).
	local z_vel is vdot(vel, zaxis).
	return v(x_vel, y_vel, z_vel).
}

declare local function get_sign {
	declare parameter value.
	if value >= 0 {
		return 1.
	} else {
		return -1.
	}
}

init_vectors().
show_vectors().

//=============================================================Pid Stuff

declare local function pid_init {
	global thruster_power is 2. // pushing power in Kn, for one direction
	global max_speed is 5.
	global max_acc is thruster_power / ship:mass.
	// Pid loop stuff for y and z translation in the final phase
	local latk is list(1, 0, 2).
	set latk[0] to latk[0] / max_acc.
	set latk[1] to latk[1] / max_acc.
	set latk[2] to latk[2] / max_acc.
	global y_pid is pidloop(latk[0], latk[1], latk[2], -1, 1).
	global z_pid is pidloop(latk[0], latk[1], latk[2], -1, 1).
}

//=============================================================Movement stuff

declare local function translate {
	declare parameter y_motion.
	declare parameter z_motion.
	set z_motion to z_motion * 1.
	set y_motion to y_motion * 1.
	//top does z axis
	//starboard does y axis
	//local top_control is ((z_motion*sin(roll_mod_1)) + (y_motion*cos(roll_mod_1))).
	//local star_control is (-(z_motion*sin(roll_mod_1)) + (y_motion*cos(roll_mod_1))).
	local star_control is 0.
	local top_control is 0.
	if roll_mod_1 = 0 {
		set star_control to (-1 * z_motion).
		set top_control to (1 * y_motion).
	} else if roll_mod_1 = 90 {
		set star_control to (1 * y_motion).
		set top_control to (1 * z_motion).
	} else if roll_mod_1 = 180 {
		set star_control to (1 * z_motion).
		set top_control to (-1 * y_motion).
	} else if roll_mod_1 = 270 {
		set star_control to (-1 * y_motion).
		set top_control to (-1 * z_motion).
	}
	set ship:control:top to top_control.
	set ship:control:starboard to star_control.
}

declare local function initial_alignment {
	// Point in the direction of the docking port
	local steering_dir is xaxis:direction + r(0,0,roll_mod_1).
	lock steering to steering_dir.
	wait until vang(xaxis, ship:facing:vector) < 1. wait 0.1.
	wait until vang(xaxis, ship:facing:vector) < 1. wait 0.1.
	wait until vang(xaxis, ship:facing:vector) < 1. wait 0.1.
}

declare local function align_yz {
	// Line up with the docking port, but not the final docking
	local steering_dir is xaxis:direction + r(0,0,roll_mod_1).
	lock steering to steering_dir.
	local pos is get_pos().
	local vel is get_velocity().
	until abs(pos:y) < 2 and abs(pos:z) < 2 {
		set pos to get_pos().
		set vel to get_velocity().
		local y_stop_dist is (vel:y^2) / (2 * max_acc).
		local y_motion is 0.
		local z_motion is 0.
		if abs(pos:y) < 0.5 and abs(vel:y) < 0.5 {
			set y_motion to 0.
		} else if abs(pos:y) < y_stop_dist and get_sign(pos:y) = get_sign(vel:y){
			set y_motion to get_sign(pos:y) * 1.
		} else if abs(vel:y) < max_speed and abs(pos:y) > 0.1 {
			set y_motion to get_sign(pos:y) * -1.
		} else if abs(vel:y) > max_speed * 1.05 {
			set y_motion to get_sign(vel:y) * 1.
		} else {
			set y_motion to 0.
		}

		local z_stop_dist is (vel:z^2) / (2 * max_acc).
		if abs(pos:z) < 0.5 and abs(vel:z) < 0.5 {
			set z_motion to 0.
		} else if abs(pos:z) < z_stop_dist and get_sign(pos:z) = get_sign(vel:z){
			set z_motion to get_sign(pos:z) * 1.
		} else if abs(vel:z) < max_speed and abs(pos:z) > 0.1 {
			set z_motion to get_sign(pos:z) * -1.
		} else if abs(vel:z) > max_speed * 1.05 {
			set z_motion to get_sign(vel:z) * 1.
		} else {
			set z_motion to 0.
		}
		translate(y_motion, z_motion).
		show_vectors().
	}
}

declare local function dock {
	// Final docking
	local steering_dir is xaxis:direction + r(0,0,roll_mod_1).
	lock steering to steering_dir.
	local pos is get_pos().
	local vel is get_velocity().
	local t is time:seconds.
	local fore_control is 0.
	local y_motion is 0.
	local z_motion is 0.
	local max_V_x is 0.5.
	local max_V_yz is 1.
	until target_port:state = "Docked (dockee)" or target_port:state = "Docked (docker)"  {
		set pos to get_pos().
		set vel to get_velocity().
		set t to time:seconds.
		// Translation in the y and z axis
		set y_motion to y_pid:update(t, pos:y).
		set z_motion to z_pid:update(t, pos:z).
		if abs(vel:y) > max_V_yz and get_sign(y_motion) <> get_sign(vel:y) {
			set y_motion to 0.
			print "Max Y speed".
		}
		if abs(vel:z) > max_V_yz and get_sign(z_motion) <> get_sign(vel:z) {
			set z_motion to 0.
			print "Max Z speed".
		}
		// Translation in the x axis
		if abs(pos:y) < 0.1 and abs(pos:z) < 0.1 {
			if abs(vel:x) < max_V_x {
				set fore_control to 1.
			} else if abs(vel:x) > max_V_x + 0.05 {
				set fore_control to -0.5.
			} else {
				set fore_control to 0.
			}
		} else {
			set fore_control to 0.
		}
		// limit speed to 1m/s when getting close
		if abs(pos:x) < 50 {
			set max_V_x to 1.
		} else {
			set max_V_x to 2.
		}
		set ship:control:fore to fore_control.
		translate(y_motion, z_motion).
		local debug is false.
		if debug {
			print "fore (x): " + ship:control:fore.
			print "star (y): " + ship:control:starboard.
			print "top  (z): " + ship:control:top.
			print "xerr:     " + pos:x.
			print "yerr:     " + pos:y.
			print "zerr:     " + pos:z.
			print vel:x.
			clearscreen.
			print target_port:state.
		}
		show_vectors().
		wait 0.05.
	}
}

rcs on.
sas off.

pid_init().

initial_alignment().
print "initial_alignment complete".
align_yz().
print "yz alignment complete".
dock().
print "docked".



set ship:control:fore to 0.
set ship:control:top to 0.
set ship:control:starboard to 0.
unlock steering.
clearvecdraws().
