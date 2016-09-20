#!/usr/bin/env python
'''
simple sailboat simulator core
'''

from aircraft import Aircraft
import util, time, math
from math import degrees, radians, sin, cos, pi, asin, atan2
from rotmat import Vector3, Matrix3
from interpolate import Interpolate

class Sailboat(Aircraft):
    '''a simple sailboat'''
    def __init__(self,
                 max_speed=5,
                 max_accel=1,
                 hull_length=1,
                 mass=3,
                 max_rudder_turn=35,
                 turning_circle=5,
                 sail_range = 70,
                 motor=False,
                 sail=True
                 ):
        Aircraft.__init__(self)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.turning_circle = turning_circle
        self.hull_length = hull_length
        self.mass = mass
        self.max_rudder_turn = max_rudder_turn
        self.last_time = time.time()
        self.have_motor = motor
        self.have_sail = sail
        self.sail_range = sail_range
        ''' Angle the simulated sail is currently at relative to rear of boat '''
        self.sail_angle = 0
        angle_atk = [0,  20, 25,  28,  30,  50,  70,  80,  90]
        lift_force = [0,  80, 110, 125, 120, 95,  70,  50,  25]
        drag_force = [10, 25, 40,  55,  65,  119, 160, 175, 180]
        self.lookup_lift = Interpolate(angle_atk, lift_force)
        self.lookup_drag = Interpolate(angle_atk, drag_force)

        angle_atk_meas =  [    0,   20,   25,   30,   35,   40,   60,   80,   90,  110,  130,  150,  170,  180]
        forward_force_p = [    0,    0,   15,   20,   20,   30,   55,   85,  105,  115,  105,  110,  140,  155]
        forward_force_s = [    0,    0,    5,   15,   30,   40,   65,  120,  150,  180,  155,  185,  165,  155]
        servo_position_p = [2012, 2012, 2000, 1964, 1980, 1940, 1746, 1514, 1466, 1184, 1184, 1184, 1184, 1184]
        servo_position_s = [2012, 2012, 2012, 2012, 1925, 1772, 1473, 1388, 1339, 1197, 1184, 1184, 1184, 1184]
        sail_position_p = [-self.servo_to_sail_angle(x) for x in servo_position_p]
        sail_position_s = [+self.servo_to_sail_angle(x) for x in servo_position_s]
        self.lookup_force = [Interpolate(angle_atk_meas, forward_force_p), Interpolate(angle_atk_meas, forward_force_s)]
        self.lookup_sail_pos = [Interpolate(angle_atk_meas, sail_position_p), Interpolate(angle_atk_meas, sail_position_s)]

    def servo_to_sail_angle(self, servo):
        # TODO: correct formula to account for non-linearity of rigging
        return (2012 - servo) / 828.0 * self.sail_range

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


    def sail_force_measured(self, sail_angle, wind_rel_dir, wind_rel_speed):
        side = 1 if wind_rel_dir > 0 else 0
        force_optimal = self.lookup_force[side][abs(wind_rel_dir)] * 9.80665002864 / 1000 # grams to newtons
        sail_optimal = self.lookup_sail_pos[side][abs(wind_rel_dir)]

        # scale force based on how far actual sail angle is from optimal
        # if > max_error degrees, zero force
        max_error = 45
        sail_error = min(1, abs(sail_optimal - sail_angle) / max_error)

        force = force_optimal * (1 - sail_error)

        print('sail_angle:%.2f wind_rel_dir:%.2f force_optimal:%.2f sail_optimal:%.2f sail_error:%.2f force:%.3f'
            % (sail_angle, wind_rel_dir, force_optimal, sail_optimal, sail_error, force))

        return force / self.mass * (wind_rel_speed * wind_rel_speed * .05)



    def sail_force_lift_drag(self, sail_angle, wind_rel_dir, wind_rel_speed):
        # TODO: replace with measured lookup table
        # angle of wind relative to sail
        angle_rel = wind_rel_dir - sail_angle
        angle_mag = abs(angle_rel)

        by_the_lee = angle_mag > 90

        if angle_mag > 180:
            angle_mag = 180
        elif angle_mag < 0:
            angle_mag = 0

        if by_the_lee:
            angle_mag = 180 - angle_mag

        lift = math.copysign(self.lookup_lift[angle_mag], angle_rel) # left+
        drag = self.lookup_drag[angle_mag] # back+

        if by_the_lee:
            lift *= 0.7

        wind_orientation = Matrix3()
        wind_orientation.from_euler(0, 0, radians(180 + wind_rel_dir))

        force_boat = wind_orientation * Vector3(drag, lift, 0)

        force_drag = wind_orientation * Vector3(drag, 0, 0)
        force_lift = wind_orientation * Vector3(0, lift, 0)

        #print('sail_angle:%.2f wind_rel_dir:%.2f mag:%.2f lift:%.2f drag:%.2f f_lift:%s f_drag:%s f:%s'
        #    % (sail_angle, wind_rel_dir, angle_mag, lift, drag, force_lift, force_drag, force_boat))

        return force_boat.x * wind_rel_speed * wind_rel_speed * 0.001


    def update(self, state):

        steering = state.steering
        throttle = state.throttle
        wind_dir = state.wind_dir
        wind_speed = state.wind_speed

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

        accel_body = Vector3(0,0,0)

        if self.have_motor:
            # target speed with current throttle
            target_speed = throttle * self.max_speed

            # linear acceleration in m/s/s - very crude model
            accel_body += Vector3(self.max_accel * (target_speed - speed) / self.max_speed, 0, 0)

        if self.have_sail:
            ### Sail calculations

            # wind from-direction to velocity vector
            wind_v = Vector3(-cos(radians(state.wind_dir)), -sin(radians(state.wind_dir)), 0) * state.wind_speed
            apparent_wind = wind_v - self.velocity
            # wind vector relative to boat body
            wind_rel_model = self.dcm.transposed() * apparent_wind
            wind_rel_dir = degrees(atan2(-wind_rel_model.y, -wind_rel_model.x))
            wind_rel_speed = wind_rel_model.length()
            #print('model_v:%s wind_v:%s apparent_v:%s rel_v:%s rel_dir: %.2f rel_spd: %.1f' % (self.velocity, wind_v, apparent_wind, wind_rel_model, wind_rel_dir, wind_rel_speed))

            ## Update sail position

            # wind relative to sail
            wind_sail_dir = wind_rel_dir - self.sail_angle
            # wrap relative to sail to determine which side sail will be pulled to
            while wind_sail_dir > 180:
                wind_sail_dir -= 360
            while wind_sail_dir <= -180:
                wind_sail_dir += 360

            sail_angle = wind_sail_dir + self.sail_angle

            # limit to movement allowed by sail servo
            max_sail_angle = self.sail_range * state.sail
            if sail_angle > max_sail_angle:
                sail_angle = max_sail_angle
            if sail_angle < -max_sail_angle:
                sail_angle = -max_sail_angle

            self.sail_angle = sail_angle

            force = self.sail_force_measured(sail_angle, wind_rel_dir, wind_rel_speed)
            accel_body += Vector3(force, 0, 0)


#        print('speed=%f throttle=%f steering=%f yaw_rate=%f accel=%f' % (speed, state.throttle, state.steering, yaw_rate, accel))

        self.gyro = Vector3(0,0,radians(yaw_rate))

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # add in accel due to direction change
        accel_body.y += radians(yaw_rate) * speed

        # now in earth frame
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)

        # drag
        accel_earth -= self.velocity * 0.1

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

