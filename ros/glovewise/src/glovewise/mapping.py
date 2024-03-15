import numpy


class KeypointMapping:

    def __init__(self, glove_model):
        def find_bone_center(glove_model, bone_name):
            center_sum = numpy.array([0.0, 0.0, 0.0], dtype=float)
            center_div = 0.0
            for skin in glove_model.skinning:
                for bone_index in range(skin.bone_count):
                    binding_matrix = skin.binding_matrices[bone_index]
                    print(binding_matrix)
                    if skin.bone_names[bone_index] == bone_name:
                        for index_index in range(len(skin.vertex_indices[bone_index])):
                            vertex_position = skin.bone_vertices[bone_index][index_index]
                            vertex_position = numpy.dot(
                                binding_matrix, vertex_position)
                            vertex_position = vertex_position[:3]
                            blend_weight = skin.bone_weights[bone_index][index_index]
                            center_sum += vertex_position * blend_weight
                            center_div += blend_weight
            if center_div > 0:
                return center_sum / center_div
            else:
                return None

        def find_bone_offset(glove_model, bone_name):
            if bone_name is None:
                return [0, 0, 0]
            center = find_bone_center(glove_model, bone_name)
            center = center * [1, 0, 1]
            return center

        def find_bone_weight(bone_name):
            if bone_name.endswith("tip") or bone_name in ["hand_thdistal", "hand_thmiddle"]:
                return 1
            return 0.2

        mapping = [
            (0, "hand_ffpalm", None, "palm"),

            (4, "hand_thtip", "hand_thmiddle", "thtip"),
            (3, "hand_thdistal", "hand_thmiddle", "thdistal"),
            (2, "hand_thmiddle", "hand_thmiddle", "thmiddle"),
            (1, "hand_thproximal", None, "thproximal"),

            (8, "hand_fftip", "hand_ffmiddle", "fftip"),
            (7, "hand_ffdistal", "hand_ffmiddle", "ffdistal"),
            (6, "hand_ffmiddle", "hand_ffmiddle", "ffmiddle"),
            (5, "hand_ffproximal", "hand_ffmiddle", "ffproximal"),

            (12, "hand_mftip", "hand_mfmiddle", "mftip"),
            (11, "hand_mfdistal", "hand_mfmiddle", "mfdistal"),
            (10, "hand_mfmiddle", "hand_mfmiddle", "mfmiddle"),
            (9, "hand_mfproximal", "hand_mfmiddle", "mfproximal"),

            (16, "hand_rftip", "hand_rfmiddle", "rftip"),
            (15, "hand_rfdistal", "hand_rfmiddle", "rfdistal"),
            (14, "hand_rfmiddle", "hand_rfmiddle", "rfmiddle"),
            (13, "hand_rfproximal", "hand_rfmiddle", "rfproximal"),

            (20, "hand_lftip", "hand_lfmiddle", "lftip"),
            (19, "hand_lfdistal", "hand_lfmiddle", "lfdistal"),
            (18, "hand_lfmiddle", "hand_lfmiddle", "lfmiddle"),
            (17, "hand_lfproximal", "hand_lfmiddle", "lfproximal"),
        ]

        chains = [
            [1, 2, 3, 4],
            [5, 6, 7, 8],
            [9, 10, 11, 12],
            [13, 14, 15, 16],
            [17, 18, 19, 20],
        ]
        connections = []
        for chain in chains:
            connections += zip(chain[:-1], chain[1:])
        self.connections = connections

        self.keypoints = [m[0] for m in mapping]
        self.links = [m[1] for m in mapping]
        self.offsets = [find_bone_offset(glove_model, m[2]) for m in mapping]
        self.weights = [find_bone_weight(m[1]) for m in mapping]

        self.robot_links = [m[3] for m in mapping]

    def keypoint_to_link(self, id):
        return self.links[self.keypoints.index(id)]
