import rospkg
import pyassimp
import lxml.etree as et
import tf.transformations
import numpy as np
import geometry_msgs.msg
import visualization_msgs.msg
from mittenwire import Profiler
import typing
import pyglovewise


class GloveSkinningNode:
    bone_count: int

    def __init__(self, node, mesh):
        self.node = node
        self.mesh = mesh
        self.bone_count = len(mesh.bones)
        self.bone_names = [bone.name for bone in mesh.bones]
        self.bone_weights = [[w.weight for w in bone.weights]
                             for bone in mesh.bones]
        self.vertex_indices = [[w.vertexid for w in bone.weights]
                               for bone in mesh.bones]
        self.binding_matrices = [bone.offsetmatrix for bone in mesh.bones]
        self.mesh_indices = mesh.faces.flatten().astype(np.int32)
        self.bone_vertices = []
        self.vertex_count = len(mesh.vertices)
        for ibone in range(self.bone_count):
            v3 = mesh.vertices[self.vertex_indices[ibone]]
            v4 = np.ones([len(v3), 4])
            v4[:, 0:3] = v3
            self.bone_vertices.append(v4)
        self.texcoords = mesh.texturecoords[0, :, 0:2]
        self.skinning = pyglovewise.Skinning()
        self.skinning.vertices = [[v[0], v[1], v[2], 1] for v in mesh.vertices]
        self.skinning.indices = self.vertex_indices
        self.skinning.weights = self.bone_weights
        self.skinning.binding = self.binding_matrices


class GloveModel:
    skinning: typing.List[GloveSkinningNode]
    model: pyassimp.structs.Scene
    link_names: typing.List[str]

    def load_file(self, file_path):
        self.model = pyassimp.core.load(file_path)
        self.model.rootnode.name = "world"
        self.skinning = []
        self.link_names = []

        def rec(node):
            self.link_names.append(node.name)
            for mesh in node.meshes:
                self.skinning.append(GloveSkinningNode(node, mesh))
            for child in node.children:
                rec(child)
        rec(self.model.rootnode)

    def load_resource(self, package_name, resource_path):
        pack = rospkg.RosPack()
        self.load_file(pack.get_path(package_name) + resource_path)

    def blend_skin_from_link_matrix_function(self, link_matrix_function):

        def transform_node(node, parent_matrix, node_matrices):
            node_matrix = tf.transformations.concatenate_matrices(
                parent_matrix, node.transformation)
            link_pose = link_matrix_function(node.name)
            if link_pose:
                node_scale, node_shear, node_angles, node_trans, node_persp = tf.transformations.decompose_matrix(
                    node_matrix)
                link_trans = link_pose[0]
                link_angles = tf.transformations.euler_from_quaternion(
                    link_pose[1])
                node_matrices[node.name] = tf.transformations.compose_matrix(
                    scale=node_scale, translate=link_trans, angles=link_angles)
            else:
                node_matrices[node.name] = node_matrix
            for child in node.children:
                transform_node(child, node_matrix, node_matrices)
        node_matrices = {}
        with Profiler("bonetrans", 0):
            transform_node(self.model.rootnode, np.identity(4), node_matrices)

        ret = []

        for skin in self.skinning:
            vertices = np.zeros([len(skin.mesh.vertices), 3])

            with Profiler("skinning2", 0):
                vertices = skin.skinning.compute(
                    [node_matrices[skin.bone_names[ibone]]
                        for ibone in range(skin.bone_count)]
                )

            with Profiler("convert", 0):
                vertices = np.array(vertices, dtype=np.float32)

            with Profiler("meshing", 0):

                ret.append(vertices[skin.mesh_indices])

        with Profiler("concat", 0):
            ret = np.concatenate(ret, axis=0)

        return ret

    def blend_skin_from_link_states(self, link_states):
        vertices = self.blend_skin_from_link_matrix_function(
            lambda link_name: link_states.link_pose(link_name).value)
        return vertices

    def blend_skin_marker_from_link_matrix_function(self, link_matrix_function):
        marker = visualization_msgs.msg.Marker()
        marker.ns = "glove"

        marker.lifetime.secs = 1
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.type = visualization_msgs.msg.Marker.TRIANGLE_LIST
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        for v in self.blend_skin_from_link_matrix_function(link_matrix_function):
            marker.points.append(
                geometry_msgs.msg.Point(x=v[0], y=v[1], z=v[2]))
        return marker

    def build_tactile_colors(self, tactile):

        ret = []
        for skin in self.skinning:

            colors = np.zeros([skin.vertex_count, 4])
            colors[:, 3] = 1

            ix = skin.texcoords[:, 0] * tactile.shape[1]
            ix = np.round(ix).astype(np.int32)
            ix = np.clip(ix, 0, tactile.shape[1] - 1)

            iy = (1 - skin.texcoords[:, 1]) * tactile.shape[0]
            iy = np.round(iy).astype(np.int32)
            iy = np.clip(iy, 0, tactile.shape[0] - 1)

            colors[:, 0:3] = tactile[iy, ix, 0:3]

            ret.append(colors[skin.mesh_indices])

        ret = np.concatenate(ret, axis=0)

        return ret

    def build_urdf(self):

        urdf = et.Element("robot", name="robot")

        def link_name(node):
            name = node.name
            return name

        def joint_name(node):
            jname = node.name
            jname = jname.split("_")[-1]
            if jname.startswith("th"):
                jname = jname[0:1] + jname[2:]
            else:
                if jname.endswith("proximal"):
                    jname = jname[0:1] + "flex"
                if jname.endswith("middle"):
                    jname = jname[0:1] + "distal"
            return jname

        def no_scaling(matrix):
            scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(
                matrix)
            return tf.transformations.compose_matrix(translate=trans, angles=angles)

        node_poses = {}

        def transform_node(node, parent_matrix):
            node_matrix = tf.transformations.concatenate_matrices(
                parent_matrix, node.transformation)
            node_pose = no_scaling(node_matrix)
            node_poses[link_name(node)] = node_pose
            for child in node.children:
                transform_node(child, node_matrix)

        transform_node(self.model.rootnode, np.identity(4))

        link_radius = 0.003

        def translate_node(node):

            if True:

                if type(node.parent) is pyassimp.structs.Scene:
                    parent_pose = np.identity(4)
                else:
                    parent_pose = node_poses[link_name(node.parent)]

                child_pose = node_poses[link_name(node)]

                relative_child_pose = tf.transformations.concatenate_matrices(
                    tf.transformations.inverse_matrix(parent_pose), child_pose)

                position = tf.transformations.translation_from_matrix(
                    relative_child_pose)
                rpy = tf.transformations.euler_from_matrix(
                    relative_child_pose, "sxyz")

                scale = tf.transformations.scale_from_matrix(
                    node.transformation)

                urdf.append(et.Comment(" %s node:%s xyz:%s rpy:%s scale:%s matrix:%s " %
                                       (link_name(node), node.name, str(position), str(rpy), str(scale), str(node.transformation).replace("\n", ""))))

                link = et.SubElement(urdf, "link", name=link_name(node))

                if type(node.parent) is not pyassimp.structs.Scene:

                    if node.parent.name not in ["world", "hand"]:
                        visual = et.SubElement(link, "visual")
                        geometry = et.SubElement(visual, "geometry")
                        et.SubElement(geometry, "sphere",
                                      radius=str(link_radius))

                    joint_parent_name = link_name(node.parent)

                    jxyz = position
                    jrpy = rpy

                    if node.name.endswith("proximal"):
                        joint_parent_name += "_a"
                        link = et.SubElement(
                            urdf, "link", name=joint_parent_name)
                        jname = joint_name(node)
                        jname = jname[0:1] + "abduct"
                        joint = et.SubElement(urdf, "joint", type="revolute",
                                              name=jname)
                        et.SubElement(joint, "origin", xyz="%f %f %f" %
                                      tuple(jxyz), rpy="%f %f %f" % tuple(jrpy))
                        et.SubElement(joint, "parent",
                                      link=link_name(node.parent))
                        et.SubElement(joint, "child", link=joint_parent_name)
                        axis = (0, 0, 1)

                        limit = [-0.5, 0.5]
                        if jname == "tabduct":
                            limit = [-1, 1.3]
                            tpos = (tf.transformations.translation_from_matrix(
                                node_poses["hand_ffproximal"]) + tf.transformations.translation_from_matrix(
                                node_poses["hand_mfproximal"])) / 2

                            fpos = tf.transformations.translation_from_matrix(
                                node_poses["hand_thproximal"])
                            p = tf.transformations.quaternion_from_matrix(
                                child_pose)
                            p = tf.transformations.quaternion_matrix(p)
                            p = tf.transformations.inverse_matrix(p)
                            axis = tuple(tf.transformations.translation_from_matrix(
                                tf.transformations.concatenate_matrices(
                                    p,
                                    tf.transformations.translation_matrix(
                                        tf.transformations.unit_vector(fpos - tpos))
                                )
                            ))
                        et.SubElement(joint, "axis", xyz="%f %f %f" % axis)
                        et.SubElement(joint, "limit", lower=str(limit[0]),
                                      upper=str(limit[1]), effort="1", velocity="1")
                        jxyz = (0, 0, 0)
                        jrpy = (0, 0, 0)

                    if node.name.endswith("proximal") or node.name.endswith("distal") or node.name.endswith("middle"):
                        jname = joint_name(node)
                        limit = [-0.2, 1.3]
                        if jname == "tdistal":
                            limit = [-1, 1]
                        axis = [-1, 0, 0]

                        if jname == "tproximal":
                            limit = [-1, 1]
                            axis = [1, 0, 0]
                        joint = et.SubElement(urdf, "joint", type="revolute",
                                              name=jname)
                        et.SubElement(joint, "origin", xyz="%f %f %f" %
                                      tuple(jxyz), rpy="%f %f %f" % tuple(jrpy))
                        et.SubElement(joint, "parent", link=joint_parent_name)
                        et.SubElement(joint, "child", link=link_name(node))
                        et.SubElement(
                            joint, "axis", xyz="%f %f %f" % tuple(axis))
                        et.SubElement(joint, "limit", lower=str(limit[0]),
                                      upper=str(limit[1]), effort="1", velocity="1")
                        if node.name.endswith("distal") and not node.name.endswith("thdistal"):
                            et.SubElement(joint, "mimic",
                                          joint=joint_name(node.parent))

                    elif node.parent.name == "hand" and node.name != "mesh":

                        joint = et.SubElement(urdf, "joint", type="floating",
                                              name=joint_name(node))

                        et.SubElement(joint, "origin",
                                      xyz="0 0 0", rpy="0 0 0")
                        et.SubElement(joint, "parent",
                                      link=link_name(node.parent))
                        et.SubElement(joint, "child", link=link_name(node))

                    else:
                        joint = et.SubElement(urdf, "joint", type="fixed",
                                              name=joint_name(node))
                        et.SubElement(joint, "origin", xyz="%f %f %f" %
                                      tuple(position), rpy="%f %f %f" % tuple(rpy))
                        et.SubElement(joint, "parent",
                                      link=link_name(node.parent))
                        et.SubElement(joint, "child", link=link_name(node))

                    if np.linalg.norm(position) > 0 and node.parent.name not in ["world", "hand"]:
                        mat = np.identity(4)
                        mat[:3, 2] = tf.transformations.unit_vector(position)
                        mat[:3, 1] = np.cross(
                            mat[:3, 2], tf.transformations.unit_vector([1, 3, 7]))
                        mat[:3, 0] = np.cross(mat[:3, 1], mat[:3, 2])

                        rpy = tf.transformations.euler_from_matrix(mat, "sxyz")

                        linename = link_name(node.parent) + "_" + \
                            link_name(node) + "_line"

                        link = et.SubElement(urdf, "link", name=linename)
                        visual = et.SubElement(link, "visual")
                        geometry = et.SubElement(visual, "geometry")
                        et.SubElement(geometry, "cylinder", radius=str(link_radius),
                                      length=str(np.linalg.norm(position)))

                        joint = et.SubElement(urdf, "joint", type="fixed",
                                              name=linename + "_joint")
                        et.SubElement(joint, "origin", xyz="%f %f %f" %
                                      tuple(position / 2), rpy="%f %f %f" % tuple(rpy))
                        et.SubElement(joint, "parent",
                                      link=link_name(node.parent))
                        et.SubElement(joint, "child", link=linename)

                for n in node.children:
                    translate_node(n)

        translate_node(self.model.rootnode)

        urdfxml = et.tostring(urdf, encoding="utf-8",
                              xml_declaration=True, pretty_print=True)

        return urdfxml.decode("utf-8")
