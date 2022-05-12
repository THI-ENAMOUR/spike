from util.logger import Logger


class ActionGroupValidator(object):
    __logger = Logger(__name__)

    def validate(self, action_group):
        """Validates an action group. Throws error if validation error occurred."""
        self.__logger.info("Validate action group " + str(action_group.id))
        # TODO: Validate stuff like start_time < end_time and contained action list is sorted by start_time
        pass
