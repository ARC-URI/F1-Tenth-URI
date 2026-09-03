"""
Main configuration file for RoboRacer.

Change ACTIVE_CONTROLLER to swap driving algorithms
without modifying main_driver.py.
"""
# from whatever python file import the class or function you want to run
from .gap_follower import GapFollower 

# our controller becomes the imported one
ACTIVE_CONTROLLER = GapFollower 