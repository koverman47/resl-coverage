from numpy.core.fromnumeric import nonzero
from numpy.lib import index_tricks
from lib_grid_map import GridMap
import numpy as np
from random import gauss


class ProbMap(GridMap):
    """[summary]

    Args:
        Input msg:  uint32 class_id
                    string class_name
                    float32 confidence *

                    # ground truth unique identifier and position of the detected object (Simulation only) 
                    uint32 object_id *
                    float32 x_pos *
                    float32 y_pos *
                    float32 z_pos *

                    # ground truth kpts projection
                    Keypoint[] kpts

                    https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros/-/blob/master/arl_unity_ros/msg/Detection.msg
    """

    def __init__(self, width_meter, height_meter, resolution,
                 center_x, center_y, init_val=0.0, false_alarm_prob=0.1):
        GridMap.__init__(self, width_meter, height_meter, resolution,
                         center_x, center_y, init_val)
        self.non_empty_cell = dict()
        # TODO need a function to set this value
        self.false_alarm_prob = false_alarm_prob

    def set_value_from_xy_index(self, index, val):
        """Stores the value in grid map

        Args:
            index (tuple): 2D tuple represents x, y coordinates.
            val (float): Value need to be stored.
        """
        self.non_empty_cell[index] = val
    
    def get_value_from_xy_index(self, index):
        """Get the value from given cell

        Args:
            index (tuple): 2D tuple represents x, y coordinates.

        Returns:
            [float]: Value from the given cell
        """
        return self.non_empty_cell[index]

    def delete_value_from_xy_index(self, index):
        """Delete the item from grid map

        Args:
            index (tuple): 2D tuple represents x, y coordinates.
        """
        del self.non_empty_cell[index]

    def map_update(self, measurement):
        """Update the probability map by received measurement

        Args:
            measurement_dict (dict): Contains measurements like {id1:[x1, y1, confidence1], id2:[x2, y2, confidence2]}
        """
        # self.set_value_from_xy_pos(meas[0], meas[1], meas[2])
        # Transfer the measurement to a dict like {(x, y):prob}, where x,y is the index of the map grid
        meas_index = dict()
        for _target_id, meas in measurement.items():
            x_pos, y_pos, meas_confidence = meas
            point_ind = tuple(self.get_xy_index_from_xy_pos(x_pos, y_pos))
            meas_index[point_ind] = meas_confidence

        for cell_ind in self.non_empty_cell.keys():
            # update all existing grids
            if cell_ind in meas_index:
                old_cell_prob = self.get_value_from_xy_index(cell_ind)
                new_cell_prob = (meas_confidence*old_cell_prob)\
                    / ((meas_confidence*old_cell_prob)+self.false_alarm_prob*(1-old_cell_prob))
                self.set_value_from_xy_index(cell_ind, new_cell_prob)
                # TODO dont delete, use flag
                # NOTE I am sure deleting a dict element is O(1) operation
                del meas_index[cell_ind]
            else:
                # if the existing grid doesn't have any update data
                # means nothing there
                meas_confidence = gauss(0.85, 0.1) # generate a reasonable prob for not sensing anything
                old_cell_prob = self.init_val
                new_cell_prob = ((1-meas_confidence)*old_cell_prob)\
                    / ((1-meas_confidence)*old_cell_prob+(1-self.false_alarm_prob)*(1-old_cell_prob))
                if new_cell_prob <= self.init_val:
                    # If the probability after update is even smaller than initial value,
                    # it's safe to delete the cell for a better memory usage
                    self.delete_value_from_xy_index(cell_ind)
                else:
                    self.set_value_from_xy_index(cell_ind, new_cell_prob)

        for cell_ind, meas_confidence in meas_index.items():
            self.set_value_from_xy_index(cell_ind, meas_confidence)

