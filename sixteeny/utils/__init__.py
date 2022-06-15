# __init__.py

from .camera import record_background, binarize, change_in_image, combine_binarized_images
from .experimenter.base import Experimenter
from .experimenter.openloop import CSVExperimenter
from .tracker import ArenaTracker
