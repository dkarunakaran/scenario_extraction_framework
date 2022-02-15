#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

class RSS:

    # Below ones are default parameter from the RSS library: https://intel.github.io/ad-rss-lib/documentation/Main.html#Section::ParameterDiscussion
    # RESPONSE TIME: 
    # common sense value for human drivers is about 2 seconds, but this can be lower for AV.
    # LONGITUDINAL ACCELERATION: 
    # Finding acceleration values for RSS is more complicated. These values may vary from ODD to ODD.
    # The minimum deceleration values must not exceed normal human driving behaviour.
    # Also big difference between the minimum and maximum acceleration will lead to defencisve driving. That leads to not participating in the dense traffic.
    # maximum acceleration a low speeds a standard car is in the range of 3.4 m/s4 to 7 m/s2
    # Required safety distance for the car at 50 km/h is brake_min = 4 m/s2 and brake_max = 8 m/s2
    # Rule of thump in german driving schools are brake_min = 4 m/s2 and brake_max = 8 m/s2
    # We are only considering one way and longitudinal safe distance. 

    def __init__(self):
        # V_ego in m/s
        # V_other in m/s

        self.res_ego = 1 #s
        self.res_other = 2 #s
        self.accel_max = 3.5 #m/s2
        self.brake_min = 4 #m/s2
        self.break_max = 8 #m/s2
        #lat_break_min = 0.8 #m/s2
        #lat_accel_max = 0.2 #m/s2
        #margin = 10 #cm

        self.vb_vel_margin = 0.25 #m/s


    def calculate_rss_safe_dist(self, V_ego, V_other):
        long_d_min = self.long_safe_dist(V_ego, V_other)

        return long_d_min

    def long_safe_dist(self, V_ego, V_other):
        first_term = V_ego*self.res_ego
        second_term = ((self.accel_max*self.res_ego**2)/2)
        third_term = (((V_ego+(self.res_ego*self.accel_max))**2)/(2*self.brake_min))
        fourth_term = ((V_other**2)/(2*self.break_max))
        d_min = first_term+second_term+third_term-fourth_term

        return d_min    


