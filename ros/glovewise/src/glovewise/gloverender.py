import rospkg
import numpy as np
import moderngl
from OpenGL import GL


class GloveRenderer:

    def __init__(self, glove_model):

        self.glove_model = glove_model

        pkg = rospkg.RosPack()
        mwpath = pkg.get_path("glovewise")

        self.gl_context = moderngl.create_standalone_context()

        self.gl_program = self.gl_context.program(
            vertex_shader=open(mwpath + "/shaders/mask_vs.glsl").read(),
            fragment_shader=open(mwpath + "/shaders/mask_fs.glsl").read()
        )

        vbo_size = 8 * sum([len(skin.mesh_indices)
                            for skin in glove_model.skinning])
        print("vbo_size", vbo_size)
        self.gl_vbo = self.gl_context.buffer(
            reserve=vbo_size)
        self.gl_vao = self.gl_context.simple_vertex_array(
            self.gl_program, self.gl_vbo, "in_vert")

        self.gl_rbo = None

    def render_mask(self, camera, link_states, image_size, roi):

        if not self.gl_rbo or self.gl_rbo.width != image_size[0] or self.gl_rbo.height != image_size[1]:
            print("creating rbo", image_size)
            self.gl_rbo = self.gl_context.renderbuffer(
                size=image_size, components=1, dtype="f1")
            self.gl_fbo = self.gl_context.framebuffer([self.gl_rbo])

        vertices = self.glove_model.blend_skin_from_link_states(link_states)

        vertices = camera.project_points_cv(vertices)
        vertices = vertices.reshape([-1, 2])

        vertices[:, 0] -= roi.x_offset - 16
        vertices[:, 1] -= roi.y_offset - 54
        vertices[:, 0] *= 1.0 / roi.width
        vertices[:, 1] *= 1.0 / roi.height

        self.gl_vbo.write(vertices.astype(np.float32).tobytes(), offset=0)

        self.gl_fbo.clear(red=0)
        self.gl_fbo.use()

        GL.glViewport(0, 0, image_size[0], image_size[1])

        self.gl_vao.render(GL.GL_TRIANGLES)

        data = self.gl_fbo.read(components=1)

        return np.frombuffer(data, dtype=np.uint8).reshape([image_size[1], image_size[0]])
