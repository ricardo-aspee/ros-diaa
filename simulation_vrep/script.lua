function sysCall_init()
    -- referencias a los objetos simulados
    youBot  = sim.getObjectHandle('/youBot')
    gps = sim.getObjectHandle('/youBot/GPS')
    
    wheelJoints= {
        sim.getObjectHandle('rollingJoint_fl'),
        sim.getObjectHandle('rollingJoint_rl'),
        sim.getObjectHandle('rollingJoint_rr'),
        sim.getObjectHandle('rollingJoint_fr'),
    
    } -- front left, rear left, rear right, front right

    -- variables asociadas a las velocidades
    forwBackVel = 0 -- velocidad longitudinal avance / retroceso
    leftRightVel = 0 -- velocidad lateral derecha / izquierda
    rotVel = 0 -- velocidad rotacional giro derecho / giro izquierdo

    -- PUBLISHER NODES
    pub_pose_base = simROS.advertise('/kuka/pose', 'geometry_msgs/Pose2D')
    simROS.publisherTreatUInt8ArrayAsString(pub_pose_base) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

    -- SUSCRIBER NODES
    base_cntrl = simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'manual_control_robot_callback')
    simROS.subscriberTreatUInt8ArrayAsString(base_cntrl)  -- treat uint8 arrays as strings   
end

function sysCall_actuation()
    -- asignacion de las velocidades segun las variables asociadas
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
end

-- esta funcion es gatillada en cada paso de la simulacion
function sysCall_sensing()
    -- obtencion de la pose desde el gps
    local objectAbsolutePosition = sim.getObjectPosition(gps,-1)
    local objectAbsoluteOrientation = sim.getObjectOrientation(gps,-1)
	
	-- generando el mensaje a publicar
    local robot_pose    = {}
    robot_pose['x']     = objectAbsolutePosition[1]
    robot_pose['y']     = objectAbsolutePosition[2]
    robot_pose['theta'] = objectAbsoluteOrientation[2]
    
	-- publicacion de la pose actual del movil
    simROS.publish(pub_pose_base, robot_pose)
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- funcion gatillada cuando se recibe un mensaje desde la suscripcion de '/cmd_vel'
function manual_control_robot_callback(msg)
    -- asignacion de las velocidades recibidas a las variables asociadas
    forwBackVel = msg.linear.x
    leftRightVel = msg.linear.y
    rotVel = msg.angular.z
end