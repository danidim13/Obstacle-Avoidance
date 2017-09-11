VFH module
==========

.. automodule:: VFH
    :members: GRID_SIZE, RESOLUTION, WINDOW_SIZE, WINDOW_CENTER, ALPHA, HIST_SIZE, THRESH, WIDE_V, V_MAX, V_MIN, OMEGA_MAX, B, A
    :undoc-members:
    :show-inheritance:

Clase VFHModel
--------------
.. currentmodule:: VFH

.. autoclass:: VFHModel

Métodos
^^^^^^^
.. autosummary::
    :toctree: VFH

    VFHModel.update_position
    VFHModel.set_target
    VFHModel.update_obstacle_density
    VFHModel.update_active_window
    VFHModel.update_polar_histogram
    VFHModel.update_filtered_polar_histogram
    VFHModel.find_valleys
    VFHModel.calculate_steering_dir
    VFHModel.calculate_speed
