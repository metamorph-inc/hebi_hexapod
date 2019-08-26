import numpy as np

from smach import State


class GetLegEffTranslations(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'],
                       input_keys=['move_base_to_leg_base_transforms','move_base_transform','leg_eff_pos_current'],
                       output_keys=['leg_eff_translations'])

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        # NOTE: Do all operation with numpy arrays for more speed... instead of lists of numpy arrays...

        # Grab userdata
        move_base_to_leg_base_tf = ud.move_base_to_leg_base_transforms
        move_base_tf = ud.move_base_transform
        leg_eff_pos_cur = [np.array([[leg[0]],[leg[1]],[leg[2]],[1]]) for leg in ud.leg_eff_pos_current]

        # Get current leg end-effector positions relative to move_base frame (mf)
        leg_eff_pos_cur_mf = [np.dot(tf,eff_pos) for tf,eff_pos in zip(move_base_to_leg_base_tf,leg_eff_pos_cur)]

        # Apply move_base_transform to current leg end-effector positions to get target leg end-effector positions
        # relative to move_base frame
        leg_eff_pos_tar_mf = [np.dot(move_base_tf,eff_pos) for eff_pos in leg_eff_pos_cur_mf]

        # Apply inverse move_base_to_leg_base_transforms to target leg end_effector_positions to get target leg end-effector positions
        # relative to leg_base frames
        leg_eff_pos_tar_lf = [np.dot(np.linalg.inv(tf),eff_pos) for tf,eff_pos in zip(move_base_to_leg_base_tf,leg_eff_pos_tar_mf)]

        # Get translations from current to target end-effector positions
        # relative to leg_base frames
        leg_eff_translations = [target-current for target,current in zip(leg_eff_pos_tar_lf,leg_eff_pos_cur)]

        # Update userdata
        # list[4x1 numpy vector] -> list[1x3 list vector]
        ud.leg_eff_translations = [[x for [x] in leg[:3, 0].reshape(3, 1).tolist()] for leg in leg_eff_translations]

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome

    def _twist_cb(self, msg):
        self._twist_msg = msg
