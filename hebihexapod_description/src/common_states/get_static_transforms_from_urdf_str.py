from smach import State

from urdf_parser_py.urdf import URDF

from pykdl_utils.kdl_kinematics import KDLKinematics


class GetStaticTransformsFromURDFStr(State):
    def __init__(self, base_links, end_links):
        """SMACH
        :type base_links: list of str
        :param base_links: Ordered list of link names in URDF (e.g. leg 1 base link, leg 2 base link, etc...)

        :type end_links: list of str
        :param end_links: Ordered list of link names in URDF (e.g. leg 1 end link, leg 2 end link, etc...)
        """
        State.__init__(self, outcomes=['done'], input_keys=['urdf_str'], output_keys=['transforms'])

        if len(base_links) > 1 and len(end_links) > 1 and len(base_links) != len(end_links):
            raise ValueError("len(base_links) > 1 and len(end_links) > 1 and len(base_links) != len(end_links)")

        self.base_links = base_links
        self.end_links = end_links

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        urdf_str = ud.urdf_str
        urdf = URDF.from_xml_string(urdf_str)

        transforms = []

        # pykdl
        kdl_list = []
        for base_link in self.base_links:
            for end_link in self.end_links:
                kdl = KDLKinematics(urdf, base_link, end_link)
                kdl_list.append(kdl)
                transform = kdl.forward([])
                transforms.append(transform)

        ud.transforms = transforms

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome
