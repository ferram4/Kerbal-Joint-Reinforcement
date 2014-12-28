using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KerbalJointReinforcement
{
    //All this class exists to do is to act as a box attached to 
    //For a sequence of three parts (A, B, C), connected in series, this will exist on B and hold the strengthening joint from A to C
    //If the joint from A to B or B to C is broken, this will destroy the joint A to C and then destroy itself
    class KJRMultiJointManager
    {
        class TwoConfigJointTuple
        {
            public ConfigurableJoint A, B;
        }

        Dictionary<PartJoint, TwoConfigJointTuple> multiJointDict;

        public KJRMultiJointManager()
        {
            multiJointDict = new Dictionary<PartJoint, TwoConfigJointTuple>();
            GameEvents.onPartJointBreak.Add(OnJointBreak);
        }

        ~KJRMultiJointManager()
        {
            Debug.Log("KJRMultiJointManager cleanup");
            GameEvents.onPartJointBreak.Remove(OnJointBreak);
        }

        public void RegisterMultiJoint(PartJoint partJoint, ConfigurableJoint multiJoint)
        {
            TwoConfigJointTuple configTuple;
            if(multiJointDict.TryGetValue(partJoint, out configTuple))
            {
                configTuple.B = multiJoint;
            }
            else
            {
                configTuple = new TwoConfigJointTuple();
                configTuple.A = multiJoint;
                multiJointDict.Add(partJoint, configTuple);
            }
        }

        private void OnJointBreak(PartJoint partJoint)
        {
            TwoConfigJointTuple configTuple;
            if (multiJointDict.TryGetValue(partJoint, out configTuple))
            {
                if (configTuple.A != null)
                    GameObject.Destroy(configTuple.A);

                if (configTuple.B != null)
                    GameObject.Destroy(configTuple.B);
                multiJointDict.Remove(partJoint);
            }
        }
    }
}
