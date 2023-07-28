import numpy as np

time  = 5
    
if __name__ == "__main__":

    steps = np.arange(0, 1000)

    position = []
    velocity = []
    acceleration = []

    dt = time/len(steps)
        
    f_pos = open('position.txt', 'w')
    f_vel = open('velocity.txt', 'w')
    f_acc = open('acceleration.txt', 'w')
        
    for step in steps:
        print(step)            
        if step < 2:
            position.append(np.array([0, 0, 0, 0]))
            velocity.append(np.array([0, 0, 0, 0]))
            acceleration.append(np.array([0, 0, 0, 0]))
            continue
        
        joint1 = 0.7 * np.pi *  np.sin(20 * np.pi * step/1000)
        joint2 = np.pi/8 + 0.01  * np.pi * np.sin(100 * np.pi * step/1000)
        joint3 = np.pi/8 + 0.1 * np.pi * np.cos(100 * np.pi * step/1000)
        joint4 = 0

        # print(joint2)
        # print(joint1, joint2, joint3, joint4)
        
        pos_joints = ([joint1, joint2, joint3, joint4])
        position.append(pos_joints)

        print(position)

        vel_joints = (np.array(position[step]) - np.array(position[step-1]))/dt
        velocity.append(list(vel_joints))

        # print(velocity    )

        acc_joints = (np.array(velocity[step]) - np.array(velocity[step-1]))/dt
        acceleration.append(list(acc_joints))

        line = ' '.join(str(x) for x in pos_joints)
        f_pos.write(line + '\n')

        line = ' '.join(str(x) for x in vel_joints)
        f_vel.write(line + '\n')

        line = ' '.join(str(x) for x in acc_joints)
        f_acc.write(line + '\n')



