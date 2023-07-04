#!/usr/bin/env python3
from beginner_tutorials.srv import AddTwoInts,SubTwoInts,MulTwoInts,DivTwoInts
import rospy

def handle_add_two_ints(req):
    print("Sum: [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return (req.a + req.b)

def handle_sub_two_ints(req):
    print("Subtraction: [%s - %s = %s]"%(req.a, req.b, (req.a - req.b)))
    return (req.a - req.b)

def handle_mul_two_ints(req):
    print("Mulitiplication [%s * %s = %s]"%(req.a, req.b, (req.a * req.b)))
    return (req.a * req.b)

def handle_div_two_ints(req):
    if req.b!=0:
        print("Division: [%s / %s = %s]"%(req.a, req.b, (req.a  /  req.b)))
        return (req.a / req.b)
    else:
        error_message = "Division by zero not allowed"
        print(error_message)
        

def arithmetic_calculations_server():
    rospy.init_node('arithmetic_calculations')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    s1 = rospy.Service('sub_two_ints', SubTwoInts, handle_sub_two_ints)
    s2=rospy.Service('mul_two_ints', MulTwoInts, handle_mul_two_ints)
    s3 = rospy.Service('div_two_ints', DivTwoInts, handle_div_two_ints)
    
    print("Ready perform arithemetic calculations.")
    print("Enter your query like this A < arithmetic operator> B. Use operators '+,-,x,/'")
    rospy.spin()

if __name__ == "__main__":
    arithmetic_calculations_server()
