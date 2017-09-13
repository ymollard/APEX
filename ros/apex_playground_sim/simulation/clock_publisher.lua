if (sim_call_type==sim_childscriptcall_initialization) then
    _pubClock=simExtRosInterface_advertise('/clock','rosgraph_msgs/Clock')
end

if (sim_call_type==sim_childscriptcall_sensing) then
    simExtRosInterface_publish(_pubClock,{clock=simGetSimulationTime()})
end

if (sim_call_type==sim_childscriptcall_cleanup) then
    simExtRosInterface_shutdownPublisher (_pubClock)
end
