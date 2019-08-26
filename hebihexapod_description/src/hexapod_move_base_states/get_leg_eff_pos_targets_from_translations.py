from smach import State


class GetLegEffPosTargetsFromTranslations(State):
    def __init__(self, translation_signs):
        """SMACH State
        :type translation_signs: list[str | None]
        :param translation_signs: E.g. ["pos", "neg", "pos", None]
        """
        State.__init__(self, outcomes=['done'],
                       input_keys=['leg_eff_translations', 'leg_eff_pos_home', 'leg_eff_pos_current'],
                       output_keys=['leg_eff_pos_targets'])

        self.translation_signs = translation_signs

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        leg_eff_translations = ud.leg_eff_translations
        leg_eff_pos_current = ud.leg_eff_pos_current
        leg_eff_pos_home = ud.leg_eff_pos_home

        if len(leg_eff_pos_home) != len(self.translation_signs):
            raise ValueError("len(leg_eff_pos_home) != len(self.translation_signs)")

        leg_eff_pos_targets = [[None, None, None]] * len(leg_eff_pos_current)

        for i, sign in enumerate(self.translation_signs):
            if sign is not None:
                if sign == "pos":
                    leg_eff_pos_targets[i] = [pos+trans for trans,pos in zip(leg_eff_translations[i],leg_eff_pos_home[i])]
                elif sign == "neg":
                    leg_eff_pos_targets[i] = [pos-trans for trans,pos in zip(leg_eff_translations[i],leg_eff_pos_home[i])]
                elif sign == "home":
                    leg_eff_pos_targets[i] = leg_eff_pos_home[i]
                else:  # in the event of some other str value
                    leg_eff_pos_targets[i] = [pos+trans for trans,pos in zip(leg_eff_translations[i],leg_eff_pos_home[i])]

        # Update userdata
        ud.leg_eff_pos_targets = leg_eff_pos_targets

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome
