"""Qt-owned scene-graph overlay migration path.

This deliberately contains no raw OpenGL/FBO objects. Stage one proves that a
registered QQuickItem can coexist with the Windows OpenGL RHI backend without
the native heap corruption caused by the legacy QQuickFramebufferObject path.
A native Qt extension is required for later raw-XYZI geometry/shader stages,
because PySide6 does not expose custom QSGGeometry attribute-set construction.
"""
from __future__ import annotations

from .contracts import OverlayRenderError, ProjectionStatus, ProjectionUniforms

try:  # Keep the geometry/controller boundary importable without PySide6.
    from PySide6.QtQml import qmlRegisterType
    from PySide6.QtQuick import QQuickItem, QSGNode

    class SceneGraphOverlayItem(QQuickItem):
        """Inert, Qt-owned scene-graph item used to validate the safe boundary."""

        supports_points = False
        backend_message = (
            "GPU overlay requires the native Qt scene-graph extension, which is "
            "not installed in this build."
        )

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self.setFlag(QQuickItem.ItemHasContents, True)
            self._status = ProjectionStatus.IDLE
            self._error: str | None = None
            self._points = None
            self._uniforms: ProjectionUniforms | None = None

        def updatePaintNode(self, old_node, update_data):  # noqa: N802 - Qt API
            # The node is fully owned by Qt's scene graph. Do not use raw gl*,
            # FBOs, QOpenGLBuffer, or QOpenGLShaderProgram in this path.
            return old_node if old_node is not None else QSGNode()

        @property
        def status(self) -> ProjectionStatus:
            return self._status

        @property
        def error_message(self) -> str | None:
            return self._error

        def set_points(self, points) -> None:
            self._points = points
            self._status = ProjectionStatus.UNAVAILABLE
            self._error = self.backend_message
            self.update()

        def set_projection_uniforms(self, uniforms: ProjectionUniforms) -> None:
            self._uniforms = uniforms
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

except ImportError:  # pragma: no cover - CI can run the Qt-free geometry core.
    class SceneGraphOverlayItem:
        supports_points = False
        backend_message = "PySide6 Qt scene-graph backend unavailable"

        def __init__(self, *args, **kwargs) -> None:
            self._status = ProjectionStatus.UNAVAILABLE
            self._error = self.backend_message

        @property
        def status(self) -> ProjectionStatus:
            return self._status

        @property
        def error_message(self) -> str:
            return self._error

        def set_points(self, points) -> None:
            pass

        def set_projection_uniforms(self, uniforms: ProjectionUniforms) -> None:
            pass

        def mark_preparing(self) -> None:
            pass

        def mark_stale(self, reason: str = "candidate envelope exceeded") -> None:
            pass

        def mark_unavailable(self, reason: str) -> None:
            self._status, self._error = ProjectionStatus.UNAVAILABLE, reason


def register_qml_type(uri: str = "LidarCalibrator.Rendering", major: int = 1, minor: int = 0) -> None:
    """Register the Qt scene-graph item; legacy FBO rendering is not registered."""
    try:
        qmlRegisterType(SceneGraphOverlayItem, uri, major, minor, "SceneGraphOverlayItem")
    except NameError as exc:  # PySide6 import path above was unavailable.
        raise OverlayRenderError("PySide6 Qt scene-graph backend unavailable") from exc
