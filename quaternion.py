import math
#---------------------------------------------------------------------------------------------

def sQuaternion_Copy(source):

   destination = { 's':source['s'],
                   'i':source['i'],
                   'j':source['j'],
                   'k':source['k']
                 }
   
   return destination

#---------------------------------------------------------------------------------------------

def Quaternion_RotationIdentity():

   q = {'s': 1.0,
        'i': 0.0,
        'j': 0.0,
        'k': 0.0
       }
   
   return q


#---------------------------------------------------------------------------------------------

def Quaternion_Multiply(lhs, rhs):

   result = {'s':(rhs['s'] * lhs['s'] - \
               rhs['i'] * lhs['i'] - \
               rhs['j'] * lhs['j'] - \
               rhs['k'] * lhs['k']),

             'i': (rhs['s'] * lhs['i'] + \
               rhs['i'] * lhs['s'] - \
               rhs['j'] * lhs['k'] + \
               rhs['k'] * lhs['j']),

             'j': (rhs['s'] * lhs['j'] + \
               rhs['i'] * lhs['k'] + \
               rhs['j'] * lhs['s'] - \
               rhs['k'] * lhs['i']),

             'k': (rhs['s'] * lhs['k'] - \
               rhs['i'] * lhs['j'] + \
               rhs['j'] * lhs['i'] + \
               rhs['k'] * lhs['s'])
            }
               
   return result

#---------------------------------------------------------------------------------------------

def Quaternion_Normalize(q):

   Magnitude = math.sqrt(
                             q['s'] * q['s'] + \
                             q['i'] * q['i'] + \
                             q['j'] * q['j'] + \
                             q['k'] * q['k'])


   q['s'] /= Magnitude;
   q['i'] /= Magnitude;
   q['j'] /= Magnitude;
   q['k'] /= Magnitude;
   
   return q

#---------------------------------------------------------------------------------------------