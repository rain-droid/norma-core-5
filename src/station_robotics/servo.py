"""Dimension-independent image feedback. Inputs are pixels, never object ground truth."""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class VisualServo:
    gain: float = .65
    step_mm: float = 5.
    path_limit_mm: float = 70.
    max_moves: int = 16
    tolerance_px: float = 4.
    matrix: np.ndarray | None = None
    offset: np.ndarray = field(default_factory=lambda: np.zeros(2))
    path_mm: float = 0.
    moves: int = 0
    stable: int = 0

    def fit(self, errors, step=4.):
        e=np.asarray(errors,float)
        J=np.column_stack([(e[1]-e[0])/step,(e[2]-e[1])/step])
        singular=np.linalg.svd(J,compute_uv=False)
        if not np.isfinite(J).all() or singular[-1]<.2 or singular[0]/singular[-1]>20:
            raise ValueError('Camera response is too weak or ambiguous; reposition and retry')
        self.matrix=J
        return J

    def correction(self, error):
        e=np.asarray(error,float)
        if e.shape!=(2,) or not np.isfinite(e).all():raise ValueError('Invalid observed image error')
        if np.linalg.norm(e)<=self.tolerance_px:
            self.stable+=1
            return None
        self.stable=0
        if self.matrix is None:raise ValueError('Observe two independent command directions first')
        if self.moves>=self.max_moves:raise ValueError('Correction count exhausted')
        delta=-self.gain*np.linalg.solve(self.matrix.T@self.matrix+.04*np.eye(2),self.matrix.T@e)
        delta*=min(1.,self.step_mm/max(np.linalg.norm(delta),1e-9))
        if self.path_mm+np.linalg.norm(delta)>self.path_limit_mm:raise ValueError('Correction path budget exhausted')
        self.path_mm+=float(np.linalg.norm(delta));self.moves+=1;self.offset+=delta
        return delta
