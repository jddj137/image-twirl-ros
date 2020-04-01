Solution to "Send a rosservice call via the command line to trigger image processing."

From the command line, enter:

    rosservice call /set_process_type flip

**Note:** If triggering the service from a different node instead of via the command line, see the name of the parameters of the request in `srv/SendString.srv`.

    set_controller_service = rospy.ServiceProxy(`/set_process_type`, ros_tutorial.srv.SendString)
    response = set_controller_service(data="flip")

