# https://github.com/bachagas/Kalman/blob/master/Kalman.h
# https://github.com/bachagas/Kalman

class USKalman:
    def __init__(self, process_noise, sensor_noise, estimated_error, intial_value):
        # The variables are x for the filtered value, q for the process noise, 
        #   r for the sensor noise, p for the estimated error and k for the Kalman Gain. 
        #   The state of the filter is defined by the values of these variables.
        #   
        #   The initial values for p is not very important since it is adjusted
        #   during the process. It must be just high enough to narrow down.
        #   The initial value for the readout is also not very important, since
        #   it is updated during the process.
        #   But tweaking the values for the process noise and sensor noise
        #   is essential to get clear readouts.
        #   
        #   For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
        #   q = 0.125
        #   r = 32
        #   p = 1023    # "large enough to narrow down"
        #   e.g.
        #   myVar = USKalman(0.125,32,1023,0)

        self.q = process_noise
        self.r = sensor_noise
        self.p = estimated_error
        self.x = intial_value         # x will hold the iterated filtered value
    
    def getFilteredValue(self, measurement):
        # Updates and gets the current measurement value 
        # prediction update
        # omit x = x
        self.p = self.p + self.q
    
        # measurement update
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p
      
        return self.x
    
    def setParameters(self, process_noise, sensor_noise, estimated_error):
        self.q = process_noise
        self.r = sensor_noise
        self.p = estimated_error

    def setParameters(self, process_noise, sensor_noise):
        self.q = process_noise
        self.r = sensor_noise
    
    def getProcessNoise(self):
        return self.q
    
    def getSensorNoise(self):
        return self.r
    
    def getEstimatedError(self):
        return self.p


