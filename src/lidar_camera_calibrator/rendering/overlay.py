"""Renderer-facing settings, uniforms, and optional Qt Quick OpenGL item.

The Qt class is intentionally a thin GPU boundary: point preparation and all
canonical chain construction remain in the Qt-free modules.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from .contracts import OverlayRenderError, OverlaySettings, ProjectionStatus, ProjectionUniforms


# GLSL 3.30 keeps the chain explicit. The CPU uploads source XYZ[I], and the
# fourth attribute is reflectance, not homogeneous w.
VERTEX_SHADER = """#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in float intensity;
uniform mat4 u_transform;
uniform mat3 u_rectification;
uniform mat3 u_intrinsics;
uniform vec3 u_rectified_translation;
uniform vec2 u_image_size;
uniform vec4 u_viewport_rect;
uniform vec2 u_framebuffer_size;
uniform float u_point_size;
out float v_depth;
out float v_intensity;
void main() {
    vec4 cam = u_transform * vec4(position, 1.0);
    vec3 rect = u_rectification * cam.xyz;
    vec3 pixel = u_intrinsics * (rect + u_rectified_translation);
    float depth = pixel.z;
    v_depth = depth;
    v_intensity = intensity;
    // NDC conversion from pixel coordinates; exact image/depth clipping is in
    // this shader so slider events only modify uniforms.
    vec2 uv = pixel.xy / depth;
    bool valid = !isnan(depth) && !isinf(depth) && depth > 0.0 && uv.x >= 0.0 && uv.x < u_image_size.x &&
                 uv.y >= 0.0 && uv.y < u_image_size.y;
    if (!valid) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); gl_PointSize = 0.0; return; }
    vec2 fbo_px = u_viewport_rect.xy + uv * u_viewport_rect.zw;
    gl_Position = vec4(2.0 * fbo_px.x / u_framebuffer_size.x - 1.0,
                       1.0 - 2.0 * fbo_px.y / u_framebuffer_size.y, 0.0, 1.0);
    gl_PointSize = u_point_size;
}
"""

FRAGMENT_SHADER = """#version 330 core
in float v_depth;
in float v_intensity;
uniform float u_depth_min;
uniform float u_depth_max;
uniform float u_opacity;
uniform int u_coloring;
out vec4 frag_color;
void main() {
    float t = clamp((v_depth - u_depth_min) / max(u_depth_max-u_depth_min, 0.0001), 0.0, 1.0);
    float value = (u_coloring == 0) ? (1.0 - t) : clamp(v_intensity, 0.0, 1.0);
    frag_color = vec4(value, 1.0-value, 0.1, u_opacity);
}
"""


try:  # Optional dependency: importing Qt-free preparation must work headlessly.
    from PySide6.QtQuick import QQuickFramebufferObject
    from PySide6.QtGui import QMatrix3x3, QMatrix4x4, QVector2D, QVector3D, QVector4D
    from PySide6.QtOpenGL import QOpenGLBuffer, QOpenGLFramebufferObject, QOpenGLShader, QOpenGLShaderProgram

    class _OverlayRenderer(QQuickFramebufferObject.Renderer):
        def __init__(self) -> None:
            super().__init__()
            self.program: QOpenGLShaderProgram | None = None
            self.point_vbo: QOpenGLBuffer | None = None
            self.points: np.ndarray | None = None
            self._points_token: int | None = None
            self._uploaded_token: int | None = None
            self.point_count = 0
            self.uniforms: ProjectionUniforms | None = None
            self.item = None

        def createFramebufferObject(self, size):  # noqa: N802 - Qt API
            return QOpenGLFramebufferObject(size)

        def synchronize(self, item) -> None:
            # synchronize() runs with the scene-graph context. CPU-side item
            # state is copied here; VBO creation/upload therefore never occurs
            # on the UI thread.
            self.item = item
            self.uniforms = item._uniforms
            self.points = item._points
            self._points_token = id(self.points)
            self.point_count = item._point_count

        def render(self) -> None:
            # A production scene may supply its own image texture. This item is
            # safe to instantiate without a current context and reports errors
            # through the item rather than crashing the UI thread.
            if self.uniforms is None or self.point_count == 0 or self.points is None:
                return
            try:
                if self.point_vbo is None:
                    self.point_vbo = QOpenGLBuffer(QOpenGLBuffer.Type.VertexBuffer)
                    if not self.point_vbo.create():
                        raise OverlayRenderError("OpenGL VBO creation failed")
                if self._uploaded_token != self._points_token:
                    if not self.point_vbo.bind():
                        raise OverlayRenderError("OpenGL VBO bind failed")
                    self.point_vbo.allocate(self.points.tobytes(), int(self.points.nbytes))
                    self._uploaded_token = self._points_token
                if self.program is None:
                    self.program = QOpenGLShaderProgram()
                    if (not self.program.addShaderFromSourceCode(QOpenGLShader.Vertex, VERTEX_SHADER)
                            or not self.program.addShaderFromSourceCode(QOpenGLShader.Fragment, FRAGMENT_SHADER)
                            or not self.program.link()):
                        raise OverlayRenderError(f"OpenGL shader setup failed: {self.program.log()}")
                self.program.bind()
                u = self.uniforms
                self.program.setUniformValue("u_transform", QMatrix4x4(*u.transform.reshape(-1).tolist()))
                self.program.setUniformValue("u_rectification", QMatrix3x3(*u.rectification.reshape(-1).tolist()))
                self.program.setUniformValue("u_intrinsics", QMatrix3x3(*u.intrinsics.reshape(-1).tolist()))
                self.program.setUniformValue("u_rectified_translation", QVector3D(*map(float, u.rectified_translation)))
                self.program.setUniformValue("u_image_size", QVector2D(float(u.image_size[0]), float(u.image_size[1])))
                self.program.setUniformValue("u_viewport_rect", QVector4D(*map(float, u.viewport_rect)))
                self.program.setUniformValue("u_framebuffer_size", QVector2D(float(u.framebuffer_size[0]), float(u.framebuffer_size[1])))
                self.program.setUniformValue("u_point_size", float(u.settings.point_size_px))
                self.program.setUniformValue("u_depth_min", float(u.settings.depth_min_metres))
                self.program.setUniformValue("u_depth_max", float(u.settings.depth_max_metres))
                self.program.setUniformValue("u_opacity", float(u.settings.opacity))
                self.program.setUniformValue("u_coloring", 0 if u.settings.coloring == "depth" else 1)
                self.point_vbo.bind()
                self.program.enableAttributeArray(0)
                self.program.enableAttributeArray(1)
                self.program.setAttributeBuffer(0, 0x1406, 0, 3, 16)  # GL_FLOAT
                self.program.setAttributeBuffer(1, 0x1406, 12, 1, 16)
                from PySide6.QtGui import QOpenGLContext
                functions = QOpenGLContext.currentContext().functions()
                # Transparent FBO: QML's PreserveAspectFit image remains visible
                # beneath the independently-owned point overlay.
                functions.glClearColor(0.0, 0.0, 0.0, 0.0)
                functions.glClear(0x00004000)  # GL_COLOR_BUFFER_BIT
                functions.glEnable(0x0BE2)  # GL_BLEND
                functions.glBlendFunc(0x0302, 0x0303)  # SRC_ALPHA, ONE_MINUS_SRC_ALPHA
                functions.glDrawArrays(0x0000, 0, self.point_count)  # GL_POINTS
                self.program.disableAttributeArray(0)
                self.program.disableAttributeArray(1)
                self.program.release()
            except Exception as exc:  # pragma: no cover - depends on host GPU
                # Do not crash the scene-graph thread; expose a typed status.
                if self.item is not None:
                    self.item._render_error(str(exc))
                return

        def releaseResources(self) -> None:  # noqa: N802 - Qt API
            # Called by Qt with the scene-graph context; resources are owned and
            # destroyed by the render thread, never by the UI-side item.
            if self.point_vbo is not None:
                self.point_vbo.destroy()
                self.point_vbo = None
            self.program = None


    class OpenGLOverlayItem(QQuickFramebufferObject):
        """Qt Quick item with a framebuffer renderer and uniform-only updates."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._uniforms: ProjectionUniforms | None = None
            self._points: np.ndarray | None = None
            self._point_count = 0
            self._status = ProjectionStatus.IDLE
            self._error: str | None = None

        def createRenderer(self):  # noqa: N802 - Qt API
            return _OverlayRenderer()

        @property
        def status(self) -> ProjectionStatus:
            return self._status

        @property
        def error_message(self) -> str | None:
            return self._error

        def set_points(self, points: np.ndarray) -> None:
            points = np.asarray(points)
            if points.ndim != 2 or points.shape[1] not in (3, 4):
                self._status, self._error = ProjectionStatus.UNAVAILABLE, "invalid point buffer shape"
                return
            if not np.isfinite(points[:, :3]).all():
                self._status, self._error = ProjectionStatus.UNAVAILABLE, "point buffer contains non-finite coordinates"
                return
            # Keep an immutable CPU copy until synchronize(); no GL resource is
            # touched from this UI-thread method.
            self._points = np.ascontiguousarray(points[:, :4] if points.shape[1] == 4 else np.column_stack((points, np.zeros(len(points), dtype=points.dtype))), dtype=np.float32)
            self._points.setflags(write=False)
            self._point_count = len(self._points)
            self._status, self._error = ProjectionStatus.READY, None
            self.update()

        def set_projection_uniforms(self, uniforms: ProjectionUniforms) -> None:
            """Update matrices/settings without touching source points or VBO."""
            self._uniforms = uniforms
            if self._point_count:
                self._status = ProjectionStatus.READY
            self.update()

        def mark_preparing(self) -> None:
            self._status = ProjectionStatus.PREPARING
            self.update()

        def mark_stale(self, reason: str = "candidate envelope exceeded") -> None:
            self._status, self._error = ProjectionStatus.STALE, reason
            self.update()

        def mark_unavailable(self, reason: str) -> None:
            self._status, self._error = ProjectionStatus.UNAVAILABLE, reason
            self.update()

        def _render_error(self, reason: str) -> None:
            self._status = ProjectionStatus.UNAVAILABLE
            self._error = f"OpenGL render failed: {reason}"
            self.update()


except ImportError:  # pragma: no cover - exercised on CI without Qt
    class OpenGLOverlayItem:
        """Safe fallback when PySide6/OpenGL is unavailable."""

        def __init__(self, *args, **kwargs) -> None:
            self._status = ProjectionStatus.UNAVAILABLE
            self._error = "PySide6 Qt Quick/OpenGL backend unavailable"
            self._point_count = 0
            self._uniforms = None

        @property
        def status(self) -> ProjectionStatus:
            return self._status

        @property
        def error_message(self) -> str:
            return self._error

        def set_points(self, points: np.ndarray) -> None:
            self._point_count = 0

        def set_projection_uniforms(self, uniforms: ProjectionUniforms) -> None:
            self._uniforms = uniforms

        def mark_preparing(self) -> None:
            self._status = ProjectionStatus.PREPARING

        def mark_stale(self, reason: str = "candidate envelope exceeded") -> None:
            self._status, self._error = ProjectionStatus.STALE, reason

        def mark_unavailable(self, reason: str) -> None:
            self._status, self._error = ProjectionStatus.UNAVAILABLE, reason


def register_qml_type(uri: str = "LidarCalibrator.Rendering", major: int = 1, minor: int = 0) -> None:
    """Register the overlay item for QML, or report an unavailable backend."""
    try:
        from PySide6.QtQml import qmlRegisterType
    except ImportError as exc:  # pragma: no cover - CI may not ship Qt
        raise OverlayRenderError("PySide6 Qt QML backend unavailable") from exc
    qmlRegisterType(OpenGLOverlayItem, uri, major, minor, "OpenGLOverlayItem")
