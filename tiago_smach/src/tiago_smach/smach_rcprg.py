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

    def shutdownRequest(self):
        self.__shutdown__ = True
