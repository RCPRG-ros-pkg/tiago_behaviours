#!/usr/bin/env python

import smach

class StateMachine(smach.StateMachine):
    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        # input_keys.append('susp_data')
        # output_keys.append('susp_data')
        # self.userdata.susp_data = ""
        smach.StateMachine.__init__(self, outcomes,
                                            input_keys=input_keys,
                                            output_keys=output_keys)

    def shutdownRequest(self):
        for ch_name, ch in self.get_children().iteritems():
            ch.shutdownRequest()

class State(smach.State):
    def __init__(self, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        input_keys.append('susp_data')
        output_keys.append('susp_data')
        smach.State.__init__(self, outcomes=outcomes,
                                        input_keys=input_keys,
                                        output_keys=output_keys,
                                        io_keys=io_keys)
        self.__shutdown__ = False
    def execute(self, userdata):
        print "STATE EXE"
        if 'susp_data' in self.get_registered_input_keys():
            if userdata.susp_data.getData() != "suspension requirements from the task harmoniser":
                print "run"
                smach.State.execute(self,userdata)
            else:
                print "run else"
                return 'SUSPENDED'
        else:
            print "else"
            smach.State.execute(self,userdata)

    def shutdownRequest(self):
        self.__shutdown__ = True
