import sys
import time
import os
import numpy as np
import logging

import helperFunctions as helpFun
# import logging_and_param as l_and_p

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


uris = [ # Drones A, B, C, D in that order
     'radio://0/80/2M/E7E7E7E7E7',
] # communication addresses

#initData = {
#    "height": 0.2
#}

if __name__ == '__main__':
    print("###################################")
    #logging.basicConfig(level=logging.DEBUG)
    #logging.basicConfig(level=logging.ERROR) #added for logging
    cflib.crtp.init_drivers(enable_debug_driver=False) # initialize drivers
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm: # create a crazyflie object linked with an uri
        #swarm.parallel(helpFun.register_consoles(swarm)) # This could potentially flood your console
        print('Resetting estimators...')
        swarm.parallel(helpFun.reset_estimator)

        print('Waiting for parameters to be downloaded...')
        #swarm.parallel(helpFun.wait_for_param_download)
        print('Initializing...')
        #helpFun.init_swarm(swarm, initData)
        print('Resetting timers...')
        #swarm.parallel(helpFun.reset_timer)
        print("###################################")
        time.sleep(0.1)
        #swarm.parallel_safe(l_and_p.log_position)

        while True:
            inp = input().split()
            """
            if (inp[0] == "quit" or inp[0] == "formation") and l_and_p.trigger_rec:
               l_and_p.save_alg() #pass
            elif (inp[0] == "formation") and not l_and_p.trigger_rec:
                # exper_dict = {}
                # exper_args = ['bcsExp', 'startExp', 1]
            
                for uri in swarm._cfs.keys():
                    exper_dict[uri] = exper_args
                swarm.parallel_safe(l_and_p.simple_param_async, args_dict= exper_dict)
                l_and_p.trigger_rec = True
                
            elif (inp[0] == "quit" or inp[1] == "debug2") and l_and_p.trigger_rec_comm:
                l_and_p.trigger_rec_comm = False
                l_and_p.save_comm()
            elif (inp[0] == "all" and inp[1] == "debug2") and not l_and_p.trigger_rec_comm:
                l_and_p.trigger_rec_comm = True
                """
            if inp == []:
                print("No input given.")
                continue

            if inp[0] == "quit":
                swarm.close_links()
                time.sleep(0.2)
                print("Program quit")
                sys.exit(0)
            elif inp[0] == "formation" and len(inp) == 2:  # usage: formation [formation name (without the ".csv")]
                #print("Running formation in 3 seconds...")
                time.sleep(.3)
                helpFun.set_formation(swarm, inp[1])
                continue

            # the scf object of the crazyflie that should execute some action, or "all"
            crazyflie = None
            if inp[0] == "all":
                crazyflie = "all"
            else:
                for uri in list(swarm._cfs):
                    if uri[-1] is inp[0]:
                        crazyflie = swarm._cfs[uri]
            if crazyflie is None:
                print(f"Crazyflie number {inp[0]} is not connected. Use a valid number (0-9) or \"all\" to address all crazyflies.")
                continue

            # the action that the crazyflie(s) should execute
            action = ""
            try:
                action = inp[1]
            except:
                print("Arguments missing")
                continue

            # creates the args_dict containing all remaining arguments, keyed with the appropriate uri(s)
            args_dict = {}
            args = inp[2:]  # list with all remaining arguments
            if crazyflie == "all":
                for uri in swarm._cfs.keys():
                    args_dict[uri] = args

            # setting desired function
            function = None

            if action == "start" and len(args) == 0:  # usage: [crazyflie] start
                function = helpFun.start
            elif action == "land" and len(args) == 0:  # usage: [crazyflie] land
                function = helpFun.land
            else:
                print("Invalid command")
                continue

            # execution of commands
            try:
                if crazyflie == "all":  # execute the command for all crazyflies
                    swarm.parallel_safe(function, args_dict=args_dict)
                else:  # execute the command for a single crazyfly
                    function(crazyflie, *args)
            except:
                print("Exeption occured")
        else:
            print("While loop exited due to some unexpected reason")



# from cflib.drivers.crazyradio import Crazyradio

# cr = Crazyradio(devid=0)
# cr.set_channel(80)
# cr.set_data_rate(cr.DR_2MPS)

# cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
# cr.set_ack_enable(False)
# cr.send_packet((0xff, 0x80, 0x63, 0x01))
# print('send')


# list(swarm._cfs): ['radio://0/80/2M/E7E7E7E7E4', 'radio://0/80/2M/E7E7E7E7E9']
# swarm._cfs.keys(): dict_keys(['radio://0/80/2M/E7E7E7E7E4', 'radio://0/80/2M/E7E7E7E7E9'])
# swarm._cfs: {'radio://0/80/2M/E7E7E7E7E9': <cflib.crazyflie.syncCrazyflie.SyncCrazyflie object at 0x000002C674794908>}
