using System;
using System.Collections;
using System.Linq;
using Unity.Robotics;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Trajectory;

namespace Unity.Robotics.UrdfImporter.Control
{
    public class RobotControl : MonoBehaviour
    {
        private ArticulationBody[] articulationChain;
        public float forceLimit;
        private readonly static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

        ROSConnection ros;
        public string topicNameOut = "/joint_states";
        public string topicNameCmd = "/joint_velocity_controller/commands";
        //public string topicTrajCmd = "/joint_states";
        public float publishMessageFrequency = 20.0f;
        //private readonly float jointAssignmentWait = 0.01f;

        private Hashtable jointPositionIndex = new Hashtable();
        public int numRobotJoints = 9;
        public string[] jointNames = {"joint_base", "joint1", "joint2", "joint3", "joint4",
                                     "joint5", "joint6", "joint7", "joint8"};

        void Start()
        {
            //ROS
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<JointStateMsg>(topicNameOut);
            ros.Subscribe<JointTrajectoryMsg>(topicNameCmd, TrajectoryHandler);
            //ros.Subscribe<JointStateMsg>(topicTrajCmd, MimicJointStates);

            //Unity
            this.gameObject.AddComponent<FKRobot>();
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            int defDyanmicVal = 10;
            foreach (ArticulationBody joint in articulationChain)
            {
                if(joint.jointType != ArticulationJointType.FixedJoint)
                {
                    //joint.gameObject.AddComponent<JointControl>();
                    joint.jointFriction = defDyanmicVal;
                    joint.angularDamping = defDyanmicVal;
                    ArticulationDrive currentDrive = joint.xDrive;
                    currentDrive.forceLimit = forceLimit;
                    currentDrive.stiffness = 10000;
                    currentDrive.damping = 100;
                    joint.xDrive = currentDrive;
                } 
            }

            for(int i = 0; i < numRobotJoints; i++)
            {
                jointPositionIndex.Add(jointNames[i], i);
            }

            Debug.Log("Starting control script...");
            InvokeRepeating("PublishJointStates", 1.0f, 1.0f/publishMessageFrequency);
        }

        private static RosMessageTypes.BuiltinInterfaces.TimeMsg now()
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            int secs = (int)(msecs / 1000);
            uint nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
            return new RosMessageTypes.BuiltinInterfaces.TimeMsg(secs, nsecs);
        }

        public void PublishJointStates()
        {
            double[] jointPositions = new double[this.numRobotJoints];
            double[] jointVelocities = new double[this.numRobotJoints];
            double[] jointEfforts = new double[this.numRobotJoints];
            
            int c = 0;
            foreach (ArticulationBody joint in articulationChain)
            {
                if (joint.jointType != ArticulationJointType.FixedJoint)
                {
                    jointPositions[c] = joint.jointPosition[0];
                    jointVelocities[c] = joint.jointVelocity[0];
                    jointEfforts[c] = joint.jointFriction;
                    c = c + 1;
                }   
            }

            HeaderMsg header = new HeaderMsg();
            header.stamp = now();
            header.frame_id = "base_link";

            JointStateMsg j_state = new JointStateMsg
            {
                header = header,
                name = this.jointNames,
                position = jointPositions,
                velocity = jointVelocities,
                effort = jointEfforts
            };

            ros.Publish(topicNameOut, j_state);
        }

        public void TrajectoryHandler(RosMessageTypes.Trajectory.JointTrajectoryMsg response)
        {
            StartCoroutine(ExecuteTrajectories(response));
        }

        private IEnumerator ExecuteTrajectories(RosMessageTypes.Trajectory.JointTrajectoryMsg joint_trajectory)
        {
            // The joint order of MoveIt is not regular so we must read the names to map
            // the right joint index.
            string[] jointNamesFromROS = joint_trajectory.joint_names;

            // For each trajectory returned by MoveIt set the xDrive and wait for completion
            for (int jointConfigIndex  = 0 ; jointConfigIndex < joint_trajectory.points.Length; jointConfigIndex++)
            {
                // Get the joint position in radians and transform to degrees
                var jointPositions = joint_trajectory.points[jointConfigIndex].positions;
                float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                var jointVelocities = joint_trajectory.points[jointConfigIndex].velocities;
                float[] resultVel = jointVelocities.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                // No points
                if (resultVel.Length == 0)
                {
                    Debug.Log("Joint trajectory returned 0 positions! Skipping");
                    continue;
                }

                int idx = 0;
                foreach (ArticulationBody joint in articulationChain)
                {
                    if(joint.jointType == ArticulationJointType.FixedJoint)
                        continue;

                    var joint1XDrive = joint.xDrive;
                    joint1XDrive.forceLimit = forceLimit;
                    joint1XDrive.stiffness = 10000;
                    joint1XDrive.damping = 100;
                    joint1XDrive.target = result[idx];
                    joint1XDrive.targetVelocity = resultVel[idx];
                    joint.xDrive = joint1XDrive;

                    idx++;
                }

                // Wait until the execution is complete
                yield return new WaitForSeconds(1.0f/publishMessageFrequency);
            }
        }

        public void MimicJointStates(RosMessageTypes.Sensor.JointStateMsg response)
        {
            string[] jointNamesFromROS = response.name;

            // Get the joint position in radians and transform to degrees
            var jointPositions = response.position;
            float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            var jointVelocities = response.velocity;
            float[] resultVel = jointVelocities.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            // Set the joint positions into the xDrive of the ArticulationBody
            /*for (int joint = 0; joint < resultVel.Length; joint++)
            {
                if(resultVel[joint] == Double.NaN)
                    continue;

                var jointName = jointNamesFromROS[joint];
                var jointIndex = (int)jointPositionIndex[jointName];
                var joint1XDrive = articulationChain[jointIndex].xDrive;
                var joint1XDrive = articulationChain[joint].xDrive;
                joint1XDrive.forceLimit = forceLimit;
                joint1XDrive.stiffness = 10000;
                joint1XDrive.damping = 100;
                joint1XDrive.target = result[joint];
                joint1XDrive.targetVelocity = resultVel[joint];  
                //articulationChain[jointIndex].xDrive = joint1XDrive;
                articulationChain[joint].xDrive = joint1XDrive;
            }*/

            int idx = 0;
            foreach (ArticulationBody joint in articulationChain)
            {
                if(joint.jointType == ArticulationJointType.FixedJoint)
                    continue;

                var joint1XDrive = joint.xDrive;
                joint1XDrive.forceLimit = forceLimit;
                joint1XDrive.stiffness = 10000;
                joint1XDrive.damping = 100;
                joint1XDrive.target = result[idx];
                joint1XDrive.targetVelocity = resultVel[idx];
                joint.xDrive = joint1XDrive;

                idx++;
            }
        }
    }
}