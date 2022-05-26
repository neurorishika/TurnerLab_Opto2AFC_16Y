# __init__.py

# from .camera import record_background, background_subtraction, binarize, change_in_image, clean_objects
from .experimenter.base import Experimenter
from .experimenter.openloop import CSVExperimenter
from .tracker import ArenaTracker
