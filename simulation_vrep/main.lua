function sysCall_init()
    youBot1  = sim.getObjectHandle('movil') 
        
    -- PUBLISHER NODES
    pub_pose_base = simROS.advertise('/kuka/pose', 'geometry_msgs/Pose2D')
    simROS.publisherTreatUInt8ArrayAsString(pub_pose_base) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
	
    -- SUSCRIBER NODES
    arm_cntrl  = simROS.subscribe('/cmd_vel_keyboard', 'geometry_msgs/Twist', 'manual_control_robot_callback')
    simROS.subscriberTreatUInt8ArrayAsString(arm_cntrl)  -- treat uint8 arrays as strings   
    
    left_motor = sim.getObjectHandle('/movil/left_motor')
    right_motor = sim.getObjectHandle('/movil/right_motor')
    
    forwBackVel=0
    leftRightVel=0
    rotVel=0
  
end
function sysCall_cleanup()
    -- do some clean-up here
end

function manual_control_robot_callback(msg)
    velocidad_longitudinal = msg.linear.x
    velocidad_lateral = msg.linear.y
    velocidad_rotacional = msg.angular.z
	
	vr = 0
	vl = 0
    
	if velocidad_longitudinal ~= 0 then
		vr = velocidad_longitudinal
		vl = velocidad_longitudinal
	-- moviendo hacia la derecha
	elseif velocidad_rotacional > 0 then
		vr = -velocidad_rotacional
		vl = velocidad_rotacional
	-- moviendo hacia la derecha
	elseif velocidad_rotacional < 0 then
		vr = velocidad_rotacional
		vl = -velocidad_rotacional
	end

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)
end
