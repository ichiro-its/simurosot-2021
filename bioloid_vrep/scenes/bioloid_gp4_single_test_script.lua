function subscriber_callback(msg)
    -- This is the subscriber callback function
    -- sim.addStatusbarMessage('subscriber receiver following Float32: '..msg.data)
end

function resetRobotPosition()
    objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    local robotobjects = sim.getObjectsInTree(objectHandle,sim.handle_all,0)
    for i = 1, 19, 1 do
        if i ~= 7 and i ~= 8 then
            tempHandle = sim.getObjectHandle("joint_0_" .. i)
            sim.setJointPosition(tempHandle, 0)
        end
    end
    for i = 1, #robotobjects, 1 do
        sim.resetDynamicObject(robotobjects[i])
    end
    sim.setObjectPosition(objectHandle, -1, {1.55, 0, 0.31349})
    sim.resetDynamicObject(objectHandle)
    
    --sim.setObjectOrientation(objectHandle, -1, {90, 0, -180})
    --sim.setObjectOrientation(objectHandle, -1, {0, 0, 0})
end

function getTransformStamped(objHandle, name, relTo, relToName)
    -- This function retrieves the stamped transform for a specific object
    t = sim.getSystemTime()
    p = sim.getObjectPosition(objHandle, relTo)
    o = sim.getObjectQuaternion(objHandle, relTo)
    return {
        header = {
            stamp = t,
            frame_id = relToName
        },
        child_frame_id = name,
        transform = {
            translation = {x = p[1], y = p[2], z = p[3]},
            rotation={x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
end

if (sim_call_type == sim.syscb_init) then
    -- The child script initialization
    objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    objectName = sim.getObjectName(objectHandle)
    
    left_foot_handle = sim.getObjectHandle("left_leg_link_5_respondable0")
    right_foot_handle = sim.getObjectHandle("right_leg_link_5_respondable0")

    --activeVisionSensor=sim.getObjectHandle('Vision_sensor')

    -- Check if the required RosInterface is there:
    moduleName = 0
    index = 0
    rosInterfacePresent = false
    while moduleName do
        moduleName = sim.getModuleName(index)
        if (moduleName == 'RosInterface') then
            rosInterfacePresent = true
        end
        index = index + 1
    end

    -- Prepare the float32 publisher and subscriber (we subscribe to the topic we advertise):
    if rosInterfacePresent then
        publisher = simROS.advertise('/simulationTime', 'std_msgs/Float32')
        img_publisher = simROS.advertise('/image', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(img_publisher)
        -- subscriber = simROS.subscribe('/simulationTime','std_msgs/Float32','subscriber_callback')
    end

end

if (sim_call_type == sim.syscb_actuation) then
    -- Send an updated simulation time message, and send the transform of the object attached to this script:
    if rosInterfacePresent then
        simROS.publish(publisher, {data = sim.getSimulationTime()})
        -- simROS.sendTransform(getTransformStamped(objectHandle,objectName,-1,'world'))
        simROS.sendTransforms({
                                getTransformStamped(objectHandle, objectName, -1, 'world'),
                                getTransformStamped(left_foot_handle, "left_foot", -1, 'world'),
                                getTransformStamped(right_foot_handle, "right_foot", -1, 'world'),
                                })
        -- To send several transforms at once, use simROS.sendTransforms instead

        -- Publish the image of the active vision sensor:
        
        --local data,w,h=sim.getVisionSensorCharImage(activeVisionSensor)
        --d={}
        --d['header']={seq=0,stamp=simROS.getTime(), frame_id="a"}
        --d['height']=h
        --d['width']=w
        --d['encoding']='rgb8'
        --d['is_bigendian']=1
        --d['step']=w*3
        --d['data']=data
        --simROS.publish(img_publisher,d)
    end
end

if (sim_call_type == sim.syscb_cleanup) then
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        -- simROS.shutdownSubscriber(subscriber)
    end
end
