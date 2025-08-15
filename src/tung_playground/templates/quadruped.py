"""Quadruped robot template for four-legged animals."""

from .base import RobotTemplate, TemplateType, PartType, TemplateLink, TemplateJoint
import math


class QuadrupedTemplate(RobotTemplate):
    """Template for quadruped (four-legged) animals like dogs, cats, horses."""
    
    def __init__(self, name: str = "quadruped_animal"):
        """Initialize quadruped template."""
        super().__init__(name, TemplateType.QUADRUPED)
    
    def _build_template(self) -> None:
        """Build the quadruped template structure."""
        # Create links
        self._create_links()
        
        # Create joints
        self._create_joints()
        
        # Set metadata
        self.metadata = {
            "description": "Quadruped animal robot template",
            "expected_parts": ["head", "torso", "front_legs", "back_legs", "tail"],
            "mobility": "quadruped_walking",
            "degrees_of_freedom": 12,  # 3 DOF per leg
            "symmetrical": True
        }
    
    def _create_links(self) -> None:
        """Create all links for the quadruped template."""
        # Base/Root link (torso/body)
        self.add_link(TemplateLink(
            name="base_link",
            part_type=PartType.TORSO,
            mass=8.0,  # Heavier body for quadruped
            inertia={"ixx": 0.2, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.3, "iyz": 0.0, "izz": 0.15}  # Elongated body
        ))
        
        # Head
        self.add_link(TemplateLink(
            name="head_link",
            part_type=PartType.HEAD,
            mass=1.5,
            origin_xyz=(0.3, 0.0, 0.1),  # Forward from torso
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.03, "iyz": 0.0, "izz": 0.02}
        ))
        
        # Tail (optional)
        self.add_link(TemplateLink(
            name="tail_link",
            part_type=PartType.TAIL,
            mass=0.5,
            origin_xyz=(-0.25, 0.0, 0.05),  # Behind torso
            inertia={"ixx": 0.005, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.01, "iyz": 0.0, "izz": 0.005}
        ))
        
        # Front Left Leg
        self.add_link(TemplateLink(
            name="front_left_hip_link",
            part_type=PartType.FRONT_LEG_LEFT,
            mass=1.0,
            origin_xyz=(0.15, -0.12, -0.05),  # Front left of body
            inertia={"ixx": 0.01, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.01, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="front_left_upper_leg_link",
            part_type=PartType.FRONT_LEG_LEFT,
            mass=1.2,
            origin_xyz=(0.0, 0.0, -0.15),  # Below hip
            inertia={"ixx": 0.015, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.015, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="front_left_lower_leg_link",
            part_type=PartType.FRONT_LEG_LEFT,
            mass=0.8,
            origin_xyz=(0.0, 0.0, -0.15),  # Below upper leg
            inertia={"ixx": 0.008, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.008, "iyz": 0.0, "izz": 0.005}
        ))
        
        self.add_link(TemplateLink(
            name="front_left_foot_link",
            part_type=PartType.FOOT_LEFT,
            mass=0.2,
            origin_xyz=(0.0, 0.0, -0.08),  # Below lower leg
            inertia={"ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.001, "iyz": 0.0, "izz": 0.001}
        ))
        
        # Front Right Leg
        self.add_link(TemplateLink(
            name="front_right_hip_link",
            part_type=PartType.FRONT_LEG_RIGHT,
            mass=1.0,
            origin_xyz=(0.15, 0.12, -0.05),  # Front right of body
            inertia={"ixx": 0.01, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.01, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="front_right_upper_leg_link",
            part_type=PartType.FRONT_LEG_RIGHT,
            mass=1.2,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.015, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.015, "iyz": 0.0, "izz": 0.01}
        ))
        
        self.add_link(TemplateLink(
            name="front_right_lower_leg_link",
            part_type=PartType.FRONT_LEG_RIGHT,
            mass=0.8,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.008, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.008, "iyz": 0.0, "izz": 0.005}
        ))
        
        self.add_link(TemplateLink(
            name="front_right_foot_link",
            part_type=PartType.FOOT_RIGHT,
            mass=0.2,
            origin_xyz=(0.0, 0.0, -0.08),
            inertia={"ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.001, "iyz": 0.0, "izz": 0.001}
        ))
        
        # Back Left Leg
        self.add_link(TemplateLink(
            name="back_left_hip_link",
            part_type=PartType.BACK_LEG_LEFT,
            mass=1.5,  # Stronger hind legs
            origin_xyz=(-0.15, -0.12, -0.05),  # Back left of body
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.015}
        ))
        
        self.add_link(TemplateLink(
            name="back_left_upper_leg_link",
            part_type=PartType.BACK_LEG_LEFT,
            mass=1.8,
            origin_xyz=(0.0, 0.0, -0.18),  # Longer hind leg segment
            inertia={"ixx": 0.025, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.025, "iyz": 0.0, "izz": 0.015}
        ))
        
        self.add_link(TemplateLink(
            name="back_left_lower_leg_link",
            part_type=PartType.BACK_LEG_LEFT,
            mass=1.0,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.012, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.012, "iyz": 0.0, "izz": 0.008}
        ))
        
        self.add_link(TemplateLink(
            name="back_left_foot_link",
            part_type=PartType.FOOT_LEFT,
            mass=0.3,
            origin_xyz=(0.0, 0.0, -0.08),
            inertia={"ixx": 0.002, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.002, "iyz": 0.0, "izz": 0.002}
        ))
        
        # Back Right Leg
        self.add_link(TemplateLink(
            name="back_right_hip_link",
            part_type=PartType.BACK_LEG_RIGHT,
            mass=1.5,
            origin_xyz=(-0.15, 0.12, -0.05),  # Back right of body
            inertia={"ixx": 0.02, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.02, "iyz": 0.0, "izz": 0.015}
        ))
        
        self.add_link(TemplateLink(
            name="back_right_upper_leg_link",
            part_type=PartType.BACK_LEG_RIGHT,
            mass=1.8,
            origin_xyz=(0.0, 0.0, -0.18),
            inertia={"ixx": 0.025, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.025, "iyz": 0.0, "izz": 0.015}
        ))
        
        self.add_link(TemplateLink(
            name="back_right_lower_leg_link",
            part_type=PartType.BACK_LEG_RIGHT,
            mass=1.0,
            origin_xyz=(0.0, 0.0, -0.15),
            inertia={"ixx": 0.012, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.012, "iyz": 0.0, "izz": 0.008}
        ))
        
        self.add_link(TemplateLink(
            name="back_right_foot_link",
            part_type=PartType.FOOT_RIGHT,
            mass=0.3,
            origin_xyz=(0.0, 0.0, -0.08),
            inertia={"ixx": 0.002, "ixy": 0.0, "ixz": 0.0,
                    "iyy": 0.002, "iyz": 0.0, "izz": 0.002}
        ))
    
    def _create_joints(self) -> None:
        """Create all joints for the quadruped template."""
        # Neck joint (head to body)
        self.add_joint(TemplateJoint(
            name="neck_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="head_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.3, 0.0, 0.1),
            lower_limit=-math.pi/4,  # -45 degrees
            upper_limit=math.pi/4,   # +45 degrees
            effort_limit=30.0,
            velocity_limit=2.0
        ))
        
        # Tail joint (optional)
        self.add_joint(TemplateJoint(
            name="tail_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="tail_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(-0.25, 0.0, 0.05),
            lower_limit=-math.pi/6,  # -30 degrees
            upper_limit=math.pi/3,   # +60 degrees (tail up)
            effort_limit=10.0,
            velocity_limit=3.0
        ))
        
        # Front Left Leg joints
        self.add_joint(TemplateJoint(
            name="front_left_hip_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="front_left_hip_link",
            axis=(1.0, 0.0, 0.0),  # Roll rotation (abduction/adduction)
            origin_xyz=(0.15, -0.12, -0.05),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=80.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_left_shoulder_joint",
            joint_type="revolute",
            parent_link="front_left_hip_link",
            child_link="front_left_upper_leg_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/3,
            upper_limit=math.pi/2,
            effort_limit=100.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_left_elbow_joint",
            joint_type="revolute",
            parent_link="front_left_upper_leg_link",
            child_link="front_left_lower_leg_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=0.0,
            upper_limit=math.pi/2,  # 90 degrees
            effort_limit=60.0,
            velocity_limit=4.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_left_ankle_joint",
            joint_type="revolute",
            parent_link="front_left_lower_leg_link",
            child_link="front_left_foot_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/6,
            upper_limit=math.pi/6,
            effort_limit=30.0,
            velocity_limit=2.0
        ))
        
        # Front Right Leg joints
        self.add_joint(TemplateJoint(
            name="front_right_hip_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="front_right_hip_link",
            axis=(1.0, 0.0, 0.0),
            origin_xyz=(0.15, 0.12, -0.05),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=80.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_right_shoulder_joint",
            joint_type="revolute",
            parent_link="front_right_hip_link",
            child_link="front_right_upper_leg_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/3,
            upper_limit=math.pi/2,
            effort_limit=100.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_right_elbow_joint",
            joint_type="revolute",
            parent_link="front_right_upper_leg_link",
            child_link="front_right_lower_leg_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=0.0,
            upper_limit=math.pi/2,
            effort_limit=60.0,
            velocity_limit=4.0
        ))
        
        self.add_joint(TemplateJoint(
            name="front_right_ankle_joint",
            joint_type="revolute",
            parent_link="front_right_lower_leg_link",
            child_link="front_right_foot_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/6,
            upper_limit=math.pi/6,
            effort_limit=30.0,
            velocity_limit=2.0
        ))
        
        # Back Left Leg joints
        self.add_joint(TemplateJoint(
            name="back_left_hip_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="back_left_hip_link",
            axis=(1.0, 0.0, 0.0),  # Roll rotation
            origin_xyz=(-0.15, -0.12, -0.05),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=120.0,  # Stronger hind legs
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_left_hip_pitch_joint",
            joint_type="revolute",
            parent_link="back_left_hip_link",
            child_link="back_left_upper_leg_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/3,
            effort_limit=150.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_left_knee_joint",
            joint_type="revolute",
            parent_link="back_left_upper_leg_link",
            child_link="back_left_lower_leg_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.18),
            lower_limit=0.0,
            upper_limit=2*math.pi/3,  # 120 degrees (powerful extension)
            effort_limit=120.0,
            velocity_limit=4.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_left_ankle_joint",
            joint_type="revolute",
            parent_link="back_left_lower_leg_link",
            child_link="back_left_foot_link",
            axis=(0.0, 1.0, 0.0),  # Pitch rotation
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=40.0,
            velocity_limit=2.0
        ))
        
        # Back Right Leg joints
        self.add_joint(TemplateJoint(
            name="back_right_hip_joint",
            joint_type="revolute",
            parent_link="base_link",
            child_link="back_right_hip_link",
            axis=(1.0, 0.0, 0.0),
            origin_xyz=(-0.15, 0.12, -0.05),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=120.0,
            velocity_limit=2.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_right_hip_pitch_joint",
            joint_type="revolute",
            parent_link="back_right_hip_link",
            child_link="back_right_upper_leg_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, 0.0),
            lower_limit=-math.pi/2,
            upper_limit=math.pi/3,
            effort_limit=150.0,
            velocity_limit=3.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_right_knee_joint",
            joint_type="revolute",
            parent_link="back_right_upper_leg_link",
            child_link="back_right_lower_leg_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.18),
            lower_limit=0.0,
            upper_limit=2*math.pi/3,
            effort_limit=120.0,
            velocity_limit=4.0
        ))
        
        self.add_joint(TemplateJoint(
            name="back_right_ankle_joint",
            joint_type="revolute",
            parent_link="back_right_lower_leg_link",
            child_link="back_right_foot_link",
            axis=(0.0, 1.0, 0.0),
            origin_xyz=(0.0, 0.0, -0.15),
            lower_limit=-math.pi/4,
            upper_limit=math.pi/4,
            effort_limit=40.0,
            velocity_limit=2.0
        ))