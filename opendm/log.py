import logging

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

# TODO add file handling

logging.addLevelName(logging.INFO, '%s[%s]' % (OKBLUE, logging.getLevelName(logging.INFO)))
logging.addLevelName(logging.WARNING, '%s[%s]' % (WARNING, logging.getLevelName(logging.WARNING)))
logging.addLevelName(logging.ERROR, '%s[%s]' % (FAIL, logging.getLevelName(logging.ERROR)))
logging.addLevelName(logging.DEBUG, '%s[%s]' % (OKGREEN, logging.getLevelName(logging.DEBUG)))

logging.basicConfig(level=logging.DEBUG,
                    format='%(levelname)-14s %(message)s' + ENDC)


def ODM_INFO(str):
    logging.info(str)


def ODM_WARNING(str):
    logging.warning(str)


def ODM_ERROR(str):
    logging.error(str)


def ODM_EXCEPTION(str):
    logging.exception(str)


def ODM_DEBUG(str):
    logging.debug(str)

