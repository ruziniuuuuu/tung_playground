"""Biped robot template for humanoid characters."""

from .base import RobotTemplate, TemplateType, PartType, TemplateLink, TemplateJoint
import math


class BipedTemplate(RobotTemplate):
    """Template for biped (two-legged) humanoid robots."""
    
    def __init__(self, name: str = "biped_humanoid"):
        """Initialize biped template."""
        super().__init__(name, TemplateType.BIPED)
    
    def _build_template(self) -> None:
        """Build the biped template structure."""
        # Create links
        self._create_links()
        
        # Create joints
        self._create_joints()
        
        # Set metadata
        self.metadata = {
            "description": "Biped humanoid robot template",
            "expected_parts": ["head", "torso", "arms", "legs", "feet", "hands"],
            "mobility": "biped_walking",
            "degrees_of_freedom": 14,  # Typical for humanoid
            "symmetrical": True
        }
    
    def _create_links(self) -> None:
        """Create all links for the biped template."""
        # Base/Root link (torso)
        self.add_link(TemplateLink(
            name="base_link",
            part_type=PartType.TORSO,
            mass=5.0,
            inertia={"ixx": 0.1, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.1, "iyz": 0.0, "izz": 0.1}
        ))
        
        # Head
        self.add_link(TemplateLink(
            name="head_link",
            part_type=PartType.HEAD,
            mass=2.0,
            origin_xyz=(0.0, 0.0, 0.3),  # Above torso
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.02}
        ))
        
        # Left Arm Chain
        self.add_link(TemplateLink(
            name="left_shoulder_link",
            part_type=PartType.ARM_LEFT,
            mass=1.5,
            origin_xyz=(-0.15, 0.0, 0.2),  # Left side of torso
            inertia={"ixx": 0.01, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.01, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="left_upper_arm_link",
            part_type=PartType.ARM_LEFT,
            mass=1.2,
            origin_xyz=(0.0, 0.0, -0.15),  # Below shoulder
            inertia={"ixx": 0.008, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.008, "iyz": 0.0, "izz": 0.008}
        ))
        
        self.add_link(TemplateLink(
            name="left_forearm_link",
            part_type=PartType.ARM_LEFT,
            mass=0.8,
            origin_xyz=(0.0, 0.0, -0.15),  # Below upper arm
            inertia={"ixx": 0.005, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.005, "iyz": 0.0, "izz": 0.005}
        ))
        
        self.add_link(TemplateLink(
            name="left_hand_link",
            part_type=PartType.HAND_LEFT,
            mass=0.3,
            origin_xyz=(0.0, 0.0, -0.08),  # Below forearm
            inertia={"ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.001, "iyz": 0.0, "izz": 0.001}
        ))
        
        # Right Arm Chain (mirrored)
        self.add_link(TemplateLink(
            name="right_shoulder_link",
            part_type=PartType.ARM_RIGHT,
            mass=1.5,
            origin_xyz=(0.15, 0.0, 0.2),  # Right side of torso
            inertia={"ixx": 0.01, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.01, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="right_upper_arm_link",
            part_type=PartType.ARM_RIGHT,
            mass=1.2,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.008, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.008, "iyz": 0.0, "izz": 0.008}
        ))
        
        self.add_link(TemplateLink(
            name="right_forearm_link",
            part_type=PartType.ARM_RIGHT,
            mass=0.8,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.005, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.005, "iyz": 0.0, "izz": 0.005}
        ))
        
        self.add_link(TemplateLink(
            name="right_hand_link",
            part_type=PartType.HAND_RIGHT,
            mass=0.3,
            origin_xyz=(0.0, 0.0, -0.08),
            inertia={"ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.001, "iyz": 0.0, "izz": 0.001}
        ))
        
        # Left Leg Chain
        self.add_link(TemplateLink(
            name="left_hip_link",
            part_type=PartType.LEG_LEFT,
            mass=2.0,
            origin_xyz=(-0.08, 0.0, -0.05),  # Below torso, left side
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.02}
        ))
        
        self.add_link(TemplateLink(
            name="left_thigh_link",
            part_type=PartType.LEG_LEFT,
            mass=3.0,
            origin_xyz=(0.0, 0.0, -0.2),  # Below hip
            inertia={"ixx": 0.03, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.03, "iyz": 0.0, "izz": 0.03}
        ))
        
        self.add_link(TemplateLink(
            name="left_shin_link",
            part_type=PartType.LEG_LEFT,
            mass=2.0,
            origin_xyz=(0.0, 0.0, -0.2),  # Below thigh
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.02}
        ))
        
        self.add_link(TemplateLink(
            name="left_foot_link",
            part_type=PartType.FOOT_LEFT,
            mass=0.8,
            origin_xyz=(0.0, 0.05, -0.05),  # Below shin, slightly forward
            inertia={"ixx": 0.005, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.005, "iyz": 0.0, "izz": 0.005}
        ))
        
        # Right Leg Chain (mirrored)
        self.add_link(TemplateLink(
            name="right_hip_link",
            part_type=PartType.LEG_RIGHT,
            mass=2.0,
            origin_xyz=(0.08, 0.0, -0.05),  # Below torso, right side
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.02}
        ))
        
        self.add_link(TemplateLink(
            name="right_thigh_link",
            part_type=PartType.LEG_RIGHT,
            mass=3.0,
            origin_xyz=(0.0, 0.0, -0.2),
            inertia={"ixx": 0.03, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.03, "iyz": 0.0, "izz": 0.03}
        ))
        
        self.add_link(TemplateLink(
            name="right_shin_link",
            part_type=PartType.LEG_RIGHT,
            mass=2.0,
            origin_xyz=(0.0, 0.0, -0.2),
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.02}
        ))
        
        self.add_link(TemplateLink(
            name="right_foot_link",
            part_type=PartType.FOOT_RIGHT,
            mass=0.8,
            origin_xyz=(0.0, 0.05, -0.05),
            inertia={"ixx": 0.005, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.005, "iyz": 0.0, "izz": 0.005}
        ))
    
    def _create_joints(self) -> None:
        """Create all joints for the biped template."""
        # Neck joint (head to torso)
        self.add_joint(TemplateJoint(
            name="neck_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="head_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, 0.3),
            lower_limit=-math.pi/3,  # -60 degrees
            upper_limit=math.pi/3,   # +60 degrees
            effort_limit=50.0,
            velocity_limit=2.0
        ))
        
        # Left arm joints
        self.add_joint(TemplateJoint(
            name="left_shoulder_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="left_shoulder_link",
            axis=(0.0, 0.0, 1.0),  # Roll rotation
            origin_xyz=(-0.15, 0.0, 0.2),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=100.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_shoulder_pitch_joint",
            joint_type="revolute",
            parent_link="left_shoulder_link",
            child_link="left_upper_arm_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=80.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_elbow_joint",
            joint_type="revolute",
            parent_link="left_upper_arm_link",
            child_link="left_forearm_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=0.0,
            upper_limit=math.pi,  # 180 degrees
            effort_limit=60.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_wrist_joint",
            joint_type="revolute",
            parent_link="left_forearm_link",
            child_link="left_hand_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=20.0,
            velocity_limit=2.0
        ))
        
        # Right arm joints (mirrored)
        self.add_joint(TemplateJoint(
            name="right_shoulder_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="right_shoulder_link",
            axis=(0.0, 0.0, 1.0),
            origin_xyz=(0.15, 0.0, 0.2),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=100.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_shoulder_pitch_joint",
            joint_type="revolute",
            parent_link="right_shoulder_link",
            child_link="right_upper_arm_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=80.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_elbow_joint",
            joint_type="revolute",
            parent_link="right_upper_arm_link",
            child_link="right_forearm_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=0.0,
            upper_limit=math.pi,
            effort_limit=60.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_wrist_joint",
            joint_type="revolute",
            parent_link="right_forearm_link",
            child_link="right_hand_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/2,
            effort_limit=20.0,
            velocity_limit=2.0
        ))
        
        # Left leg joints
        self.add_joint(TemplateJoint(
            name="left_hip_yaw_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="left_hip_link",
            axis=(0.0, 0.0, 1.0),  # Yaw rotation
            origin_xyz=(-0.08, 0.0, -0.05),
            lower_limit=-math.pi/6,  # -30 degrees
            upper_limit=math.pi/6,   # +30 degrees
            effort_limit=150.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_hip_pitch_joint",
            joint_type="revolute",
            parent_link="left_hip_link",
            child_link="left_thigh_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/3,
            effort_limit=200.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_knee_joint",
            joint_type="revolute",
            parent_link="left_thigh_link",
            child_link="left_shin_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.2),
            lower_limit=0.0,
            upper_limit=math.pi/2,  # 90 degrees
            effort_limit=150.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="left_ankle_joint",
            joint_type="revolute",
            parent_link="left_shin_link",
            child_link="left_foot_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.2),
            lower_limit=-math.pi/6,
            upper_limit=math.pi/6,
            effort_limit=80.0,
            velocity_limit=2.0
        ))
        
        # Right leg joints (mirrored)
        self.add_joint(TemplateJoint(
            name="right_hip_yaw_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="right_hip_link",
            axis=(0.0, 0.0, 1.0),
            origin_xyz=(0.08, 0.0, -0.05),
            lower_limit=-math.pi/6,
            upper_limit=math.pi/6,
            effort_limit=150.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_hip_pitch_joint",
            joint_type="revolute",
            parent_link="right_hip_link",
            child_link="right_thigh_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/3,
            effort_limit=200.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_knee_joint",
            joint_type="revolute",
            parent_link="right_thigh_link",
            child_link="right_shin_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.2),
            lower_limit=0.0,
            upper_limit=math.pi/2,
            effort_limit=150.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="right_ankle_joint",
            joint_type="revolute",
            parent_link="right_shin_link",
            child_link="right_foot_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.2),
            lower_limit=-math.pi/6,
            upper_limit=math.pi/6,
            effort_limit=80.0,
            velocity_limit=2.0
        ))