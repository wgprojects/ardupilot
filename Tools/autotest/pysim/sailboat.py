#!/usr/bin/env python
'''
simple sailboat simulator core
'''

from aircraft import Aircraft
import util, time, math
from math import degrees, radians, sin, cos, pi, asin, atan2
from rotmat import Vector3, Matrix3

class Sailboat(Aircraft):
    '''a simple sailboat'''
    def __init__(self,
                 max_speed=5,
                 max_accel=1,
                 hull_length=1,
                 max_rudder_turn=35,
                 turning_circle=2,
                 ):
        Aircraft.__init__(self)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.turning_circle = turning_circle
        self.hull_length = hull_length
        self.max_rudder_turn = max_rudder_turn
        self.last_time = time.time()

    def turn_circle(self, steering):
        '''return turning circle (diameter) in meters for steering angle proportion in degrees
        '''
        if abs(steering) < 1.0e-6:
            return 0
        return self.turning_circle * sin(radians(self.max_rudder_turn)) / sin(radians(steering*self.max_rudder_turn))

    def yaw_rate(self, steering, speed):
        '''return yaw rate in degrees/second given steering_angle and speed'''
        if abs(steering) < 1.0e-6 or abs(speed) < 1.0e-6:
            return 0
        d = self.turn_circle(steering)
        c = pi * d
        t = c / speed
        rate = 360.0 / t
        return rate

    def update(self, state):

        steering = state.steering
        throttle = state.throttle

        # wind from-direction to velocity vector
        wind_v = Vector3(-cos(radians(state.wind_dir)), -sin(radians(state.wind_dir)), 0) * state.wind_speed
        apparent_wind = wind_v - self.velocity
        # wind vector relative to boat body
        wind_rel_model = self.dcm.transposed() * apparent_wind;
        wind_rel_dir = degrees(atan2(-wind_rel_model.y, -wind_rel_model.x));
        wind_rel_speed = wind_rel_model.length();
        #print('model_v:%s wind_v:%s apparent_v:%s rel_v:%s rel_dir: %.2f rel_spd: %.1f' % (self.velocity, wind_v, apparent_wind, wind_rel_model, wind_rel_dir, wind_rel_speed))


        # TODO: SAIL SIMULATION GOES HERE
        max_sail_angle = 90 * state.sail;
        # - actual position from wind direction
        # math.copysign(max_sail_swing, wind_rel_model.y);
        # - lookup table force = f(sail_angle, wind_rel_dir, wind_rel_speed)
        # - acceleration = force / mass
        # - drag, etc

        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # speed in m/s in body frame
        velocity_body = self.dcm.transposed() * self.velocity

        # speed along x axis, +ve is forward
        speed = velocity_body.x

        # yaw rate in degrees/s
        yaw_rate = self.yaw_rate(steering, speed)

        # target speed with current throttle
        target_speed = throttle * self.max_speed

        # linear acceleration in m/s/s - very crude model
        accel = self.max_accel * (target_speed - speed) / self.max_speed

#        print('speed=%f throttle=%f steering=%f yaw_rate=%f accel=%f' % (speed, state.throttle, state.steering, yaw_rate, accel))

        self.gyro = Vector3(0,0,radians(yaw_rate))

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # accel in body frame due to motor
        accel_body = Vector3(accel, 0, 0)

        # add in accel due to direction change
        accel_body.y += radians(yaw_rate) * speed

        # now in earth frame
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)

        # if we're on the ground, then our vertical acceleration is limited
        # to zero. This effectively adds the force of the ground on the aircraft
        accel_earth.z = 0

        # work out acceleration as seen by the accelerometers. It sees the kinematic
        # acceleration (ie. real movement), plus gravity
        self.accel_body = self.dcm.transposed() * (accel_earth + Vector3(0, 0, -self.gravity))

        # new velocity vector
        self.velocity += accel_earth * delta_time

        # new position vector
        old_position = self.position.copy()
        self.position += self.velocity * delta_time

        # update lat/lon/altitude
        self.update_position(delta_time)

if __name__ == "__main__":
    r = Sailboat()
    d1 = r.turn_circle(r.max_rudder_turn)
    print("turn_circle=", d1)

