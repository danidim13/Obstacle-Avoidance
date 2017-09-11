VFHP module
===========

.. automodule:: VFHP
    :members: GRID_SIZE, C_MAX, RESOLUTION, WINDOW_SIZE, WINDOW_CENTER, ALPHA, HIST_SIZE, B, A, R_ROB, T_LO, T_HI, WIDE_V, V_MAX, V_MIN, mu1, mu2, mu3, MAX_COST
    :undoc-members:
    :show-inheritance:

Clase VFHPModel
---------------
.. currentmodule:: VFHP

.. autoclass:: VFHPModel

Métodos
^^^^^^^
.. autosummary::
    :toctree: VFHP

    VFHPModel.update_position
    VFHPModel.set_target
    VFHPModel.update_obstacle_density
    VFHPModel.update_active_window
    VFHPModel.update_polar_histogram
    VFHPModel.update_bin_polar_histogram
    VFHPModel.update_masked_polar_hist
    VFHPModel.find_valleys
    VFHPModel.calculate_steering_dir

