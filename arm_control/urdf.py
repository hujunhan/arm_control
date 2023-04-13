# coding= utf8
"""
This module contains the main functions used to parse URDF files.
"""

import xml.etree.ElementTree as ET
import warnings
from loguru import logger
import math
import pathlib


class URDF:
    def __init__(self):
        self.joint_list = (
            []
        )  # List of joints, contains 'name', 'joint_type', 'origin_translation', 'origin_orientation', 'rotation', 'translation', 'bounds'
        self.link_list = (
            []
        )  # List of links, contains 'name', 'collision_mesh_path', 'visual_mesh_path'
        pass

    @staticmethod
    def _find_next_joint(root, current_link, next_joint_name):
        """
        Find the next joint in the URDF tree

        Parameters
        ----------
        root
        current_link: xml.etree.ElementTree
            The current URDF link
        next_joint_name: str
            Optional : The name of the next joint. If not provided, find it automatically as the first child of the link.

        Returns
        -------
        has_next: bool
            True if the next joint has been found. Otherwise, the next joint will be None
        next_joint: Optional[Element]
            The next joint
        """
        # Find the joint attached to the link
        has_next = False
        next_joint = None
        search_by_name = next_joint_name is not None
        current_link_name = None

        if not search_by_name:
            # If no next joint is provided, find it automatically
            current_link_name = current_link.attrib["name"]

        # FIXME: Use a filter expression directly in the findall statement, instead of filtering by hand
        for joint in root.findall("joint"):
            # Only use joints and links defined at the root.
            # There may be other joints and links elsewhere, but they are just metadata
            if joint is not None:
                # Iterate through all joints to find the good one
                if search_by_name:
                    # Find the joint given its name
                    if joint.attrib["name"] == next_joint_name:
                        has_next = True
                        next_joint = joint
                        break

                else:
                    # Find the first joint whose parent is the current_link
                    # FIXME: We are not sending a warning when we have two children for the same link
                    # Even if this is not possible, we should ensure something coherent
                    if joint.find("parent").attrib["link"] == current_link_name:
                        has_next = True
                        next_joint = joint
                        break

        if search_by_name and not has_next:
            raise ValueError(
                "Error: joint {} given but not found in the URDF".format(
                    next_joint_name
                )
            )

        return has_next, next_joint

    @staticmethod
    def _find_next_link(root, current_joint, next_link_name):
        """
        Find the next link in the URDF tree

        Parameters
        ----------
        root
        current_joint: xml.etree.ElementTree
            The current URDF joint
        next_link_name: str
            Optional : The name of the next link. If not provided, find it automatically as the first child of the joint.

        Returns
        -------
        has_next: bool
            True if the next link has been found. Otherwise, the next link will be None
        next_link: Optional[Element]
            The next link
        """
        has_next = False
        next_link = None

        given_next_link = next_link_name is not None

        # If no next link, find it automatically
        if next_link_name is None:
            # If the name of the next link is not provided, find it
            next_link_name = current_joint.find("child").attrib["link"]

        # FIXME: Directly find the link using a regex and a filter
        for urdf_link in root.findall("link"):
            if urdf_link.attrib["name"] == next_link_name:
                next_link = urdf_link
                has_next = True

        if given_next_link and not has_next:
            raise ValueError(
                "Error: link {} given but not found in the URDF".format(next_link_name)
            )

        return has_next, next_link

    @staticmethod
    def _find_parent_link(root, joint_name):
        """Find the first link which is the parent of the given joint"""
        try:
            parent_link = next(
                joint.find("parent").attrib["link"]
                for joint in root.iter("joint")
                if joint.attrib["name"] == joint_name
            )
        except StopIteration:
            raise ValueError("Unable to locate the parent link")

        return parent_link

    def get_urdf_parameters(
        self,
        urdf_file,
        base_elements=None,
        last_link_vector=None,
        base_element_type="link",
        symbolic=True,
    ):
        """
        Returns translated parameters from the given URDF file.
        Parse the URDF joints into IKPY links, throw away the URDF links.

        Parameters
        ----------
        urdf_file: str
            The path of the URDF file
        base_elements: list of strings
            List of the links beginning the chain
        last_link_vector: numpy.array
            Optional : The translation vector of the tip.
        base_element_type: str
        symbolic: bool

        Returns
        -------
        list[ikpy.link.URDFLink]
        """
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        # base_elements = list(base_elements)
        if base_elements is None:
            base_elements = ["base_link"]
        elif base_elements is []:
            raise ValueError("base_elements can't be the empty list []")

        joints = []
        links = []
        has_next = True
        current_joint = None
        current_link = None

        # Initialize the tree traversal
        if base_element_type == "link":
            # The first element is a link, so its (virtual) parent should be a joint
            node_type = "joint"
        elif base_element_type == "joint":
            # The same as before, but swap link and joint
            node_type = "link"
        else:
            raise ValueError("Unknown type: {}".format(base_element_type))

        # Parcours récursif de la structure de la chain
        while has_next:
            if len(base_elements) != 0:
                next_element = base_elements.pop(0)
            else:
                next_element = None

            if node_type == "link":
                # Current element is a link, find child joint
                (has_next, current_joint) = self._find_next_joint(
                    root, current_link, next_element
                )
                node_type = "joint"
                if has_next:
                    joints.append(current_joint)
                    logger.debug(
                        "Next element: joint {}".format(current_joint.attrib["name"])
                    )

            elif node_type == "joint":
                # Current element is a joint, find child link
                (has_next, current_link) = self._find_next_link(
                    root, current_joint, next_element
                )
                node_type = "link"
                if has_next:
                    links.append(current_link)
                    logger.debug(
                        "Next element: link {}".format(current_link.attrib["name"])
                    )

        parameters = []

        # Save the links in the good format
        for link in links:
            link_info = {}
            link_info["name"] = link.attrib["name"]
            if link.find("visual") is not None:
                mesh = link.find("visual").find("geometry").find("mesh")
                if mesh is not None:
                    relative_mesh_filename = mesh.attrib["filename"]
                    urdf_path = pathlib.Path(urdf_file)
                    absolute_mesh_filename = (
                        urdf_path.parent / relative_mesh_filename
                    ).absolute()
                    link_info["visual_mesh_path"] = absolute_mesh_filename
                else:
                    logger.warning(f'No mesh found for link "{link.attrib["name"]}"')
            if link.find("collision") is not None:
                mesh = link.find("collision").find("geometry").find("mesh")
                if mesh is not None:
                    relative_mesh_filename = mesh.attrib["filename"]
                    urdf_path = pathlib.Path(urdf_file)
                    absolute_mesh_filename = (
                        urdf_path.parent / relative_mesh_filename
                    ).absolute()
                    link_info["collision_mesh_path"] = absolute_mesh_filename
                else:
                    logger.warning(f'No mesh found for link "{link.attrib["name"]}"')
            self.link_list.append(link_info)

        # Save the joints in the good format
        for joint in joints:
            joint_info = {}
            origin_translation = [0, 0, 0]
            origin_orientation = [0, 0, 0]
            rotation = None
            translation = None
            bounds = [float("-inf"), float("inf")]

            origin = joint.find("origin")
            if origin is not None:
                if "xyz" in origin.attrib.keys():
                    origin_translation = [
                        float(x) for x in origin.attrib["xyz"].split()
                    ]
                if "rpy" in origin.attrib.keys():
                    origin_orientation = [
                        float(x) for x in origin.attrib["rpy"].split()
                    ]

            joint_type = joint.attrib["type"]

            if joint_type not in [
                "revolute",
                "prismatic",
                "fixed",
                "continuous",
                "planar",
                "floating",
                "fixed",
            ]:
                raise ValueError("Unknown joint type: {}".format(joint_type))

            axis = joint.find("axis")
            if axis is not None:
                if joint_type == "revolute" or joint_type == "continuous":
                    rotation = [float(x) for x in axis.attrib["xyz"].split()]
                    translation = None
                elif joint_type == "prismatic":
                    rotation = None
                    translation = [float(x) for x in axis.attrib["xyz"].split()]
                elif joint_type == "fixed":
                    warnings.warn(
                        "Joint {} is of type: fixed, but has an 'axis' attribute defined. This is not in the URDF spec and thus this axis is ignored".format(
                            joint.attrib["name"]
                        )
                    )
                else:
                    raise ValueError(
                        "Unknown joint type with an axis: {}, {}".format(
                            joint_type, axis
                        )
                    )

            limit = joint.find("limit")
            if limit is not None:
                if "lower" in limit.attrib:
                    bounds[0] = float(limit.attrib["lower"])
                if "upper" in limit.attrib:
                    bounds[1] = float(limit.attrib["upper"])

            joint_info["name"] = joint.attrib["name"]
            joint_info["joint_type"] = joint_type

            joint_info["origin_translation"] = origin_translation
            joint_info["origin_orientation"] = origin_orientation
            joint_info["rotation"] = rotation
            joint_info["translation"] = translation
            joint_info["bounds"] = bounds

            ## add info to list

            self.joint_list.append(joint_info)
            # print(frame_matrix)

        # Add last_link_vector to parameters
        if last_link_vector is not None:
            pass

        return parameters


if __name__ == "__main__":
    file_path = "./urdf/ur10.urdf"
    urdf = URDF()
    urdf.get_urdf_parameters(file_path)
    for joint in urdf.joint_list:
        logger.info(joint)
        # pass
