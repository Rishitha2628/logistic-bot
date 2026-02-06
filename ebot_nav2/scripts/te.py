import math 

def convert_to_signed_angle(angle):
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
p = convert_to_signed_angle( 2.55)
print(p)