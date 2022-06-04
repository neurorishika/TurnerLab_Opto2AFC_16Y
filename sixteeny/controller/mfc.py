import alicat

class MFCController(object):
    """
    A class to control the Alicat MFC for the Y Maze
    """
    def __init__(self, com_port, device_ids, default_flow_rate):
        self.com_port = com_port
        self.device_ids = device_ids
        self.mfcs = []
        for i in range(len(self.device_ids)):
            self.mfcs.append(alicat.FlowController(port=self.com_port, address=self.device_ids[i]))
        self.default_flow_rate = default_flow_rate
        
    def init(self):
        """
        Initialise the MFCs
        """
        for i in range(len(self.mfcs)):
            if self.test_connection(i):
                self.set_flow_rate(i, self.default_flow_rate)
            else:
                raise Exception('No Alicat MFC with ID: {} found on COM{}'.format(self.device_ids[i], self.com_port))

    def __enter__(self):
        """
        Initialise the MFCs with a context manager
        """
        self.init()
        return self

    def close(self):
        """
        Close the MFCs
        """
        for i in range(len(self.mfcs)):
            # set flow rate to 0
            # self.set_flow_rate(i, 0)
            # close the connection
            self.mfcs[i].close()
        
    def __exit__(self, type, value, traceback):
        """
        Close the MFCs with a context manager
        """
        self.close()
        
    def test_connection(self,index):
        """
        Test the connection to all the MFCs
        """
        try:
            self.mfcs[index].get()
            return True
        except:
            return False
    
    def set_flow_rate(self, index, flow_rate):
        """
        Set the flow rate of the MFC
        """
        self.mfcs[index].set_flow_rate(flow_rate)
    
    def get_flow_rate(self, index):
        """
        Get the flow rate of the MFC
        """
        return self.mfcs[index].get()['volumetric_flow']

    def get_all_properties(self, index):
        """
        Get all the properties of the MFC
        """
        return self.mfcs[index].get()
        