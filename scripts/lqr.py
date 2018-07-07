import numpy as np
import scipy as sp
import scipy.linalg as splinalg
import control as ctrl

m_pole = 2.0
m_car = 1.0
l = 0.6
theta = np.pi
g = 9.8

B = np.array([[1],[0]])

M = np.array([[m_car + m_pole, m_pole * l * np.cos(theta)],
              [m_pole * l * np.cos(theta), m_pole * l * l]])


dt_dq = np.array([[0, 0],
                  [0, -m_pole * g * l * np.cos(theta)]])

A_t_part = np.matmul( np.linalg.inv(M) , dt_dq )

B_t_part = np.matmul( np.linalg.inv(M) , B )

A_11 = np.zeros(np.shape(A_t_part))
A_12 = np.eye( np.shape(A_t_part)[0], np.shape(A_t_part)[1] )
A_21 = A_t_part
A_22 = np.zeros(np.shape(A_t_part))
A_1 = np.hstack( (A_11, A_12) )
A_2 = np.hstack( (A_21, A_22) )
A_t = np.vstack( (A_1, A_2) )

B_1 = np.zeros(np.shape(B_t_part))
B_t = np.vstack( (B_1, B_t_part) )


Q = np.array([[20, 0, 0, 0],
              [0, 20, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R = np.array([[1]])

K, S, E = ctrl.lqr(A_t, B_t, Q, R)
print(K)
print(S)



# import argparse
# import math
# import numpy as np
#
# from pydrake.all import (BasicVector, DiagramBuilder, FloatingBaseType,
#                          LinearQuadraticRegulator, RigidBodyPlant,
#                          RigidBodyTree, Simulator)
# from underactuated import (FindResource, PlanarRigidBodyVisualizer)
#
#
# def UprightState():
#     state = (0, math.pi, 0, 0)
#     return state
#
#
# def BalancingLQR(robot):
#     # Design an LQR controller for stabilizing the CartPole around the upright.
#     # Returns a (static) AffineSystem that implements the controller (in
#     # the original CartPole coordinates).
#
#     context = robot.CreateDefaultContext()
#     context.FixInputPort(0, BasicVector([0]))
#
#     context.get_mutable_continuous_state_vector().SetFromVector(UprightState())
#
#     Q = np.diag((10., 10., 1., 1.))
#     R = [1]
#
#     return LinearQuadraticRegulator(robot, context, Q, R)


# if __name__ == "__main__":
#     builder = DiagramBuilder()
#
#     tree = RigidBodyTree(FindResource("cartpole/cartpole.urdf"),
#                          FloatingBaseType.kFixed)
#
#     robot = builder.AddSystem(RigidBodyPlant(tree))
#     controller = builder.AddSystem(BalancingLQR(robot))
#     builder.Connect(robot.get_output_port(0), controller.get_input_port(0))
#     builder.Connect(controller.get_output_port(0), robot.get_input_port(0))
#
#     visualizer = builder.AddSystem(PlanarRigidBodyVisualizer(tree,
#                                                              xlim=[-2.5, 2.5],
#                                                              ylim=[-1, 2.5]))
#     builder.Connect(robot.get_output_port(0), visualizer.get_input_port(0))
#
#     diagram = builder.Build()
#     simulator = Simulator(diagram)
#     simulator.set_target_realtime_rate(1.0)
#     simulator.set_publish_every_time_step(False)
#     context = simulator.get_mutable_context()
#
#     state = context.get_mutable_continuous_state_vector()
#
#     parser = argparse.ArgumentParser()
#     parser.add_argument("-N", "--trials",
#                         type=int,
#                         help="Number of trials to run.",
#                         default=5)
#     parser.add_argument("-T", "--duration",
#                         type=float,
#                         help="Duration to run each sim.",
#                         default=10.0)
#     args = parser.parse_args()
#
#     for i in range(args.trials):
#         context.set_time(0.)
#         state.SetFromVector(UprightState() + 0.1*np.random.randn(4,))
#         simulator.StepTo(args.duration)

















# end
