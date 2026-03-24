import os
import numpy as np

class _BaseTrainingDS:
    """Base dataset class for handling input and label data."""

    def __init__(self,
                 dataset_path: str,
                 input_folder: str = "inputs",
                 label_folder: str = "labels",
                 ) -> None:
        """Initialize the base training dataset.

        Args:
            dataset_path (str): Path to the dataset directory.
            input_folder (str): Name of the folder containing input data.
            label_folder (str): Name of the folder containing label data.
        """
        # input folder
        self.inputs_enabled = False
        self.input_folder = input_folder
        self.input_files = []

        # labels folder
        self.labels_enabled = False
        self.label_folder = label_folder
        self.label_files = []

        # variable to keep track of the number of frames
        self.num_frames = 0

        # load the new dataset
        self.load_new_dataset(dataset_path)

    def load_new_dataset(self, dataset_path: str) -> None:
        """Load a new dataset from a path.

        Args:
            dataset_path (str): The path to the new dataset.
        """
        self.dataset_path = dataset_path
        self.import_dataset_files()
        self.determine_num_frames()

    def import_dataset_files(self) -> None:
        """Import input and label files."""
        self.import_input_data()
        self.import_label_data()

    def determine_num_frames(self) -> None:
        """Determine the total number of frames in the dataset."""
        self.num_frames = 0

        if self.inputs_enabled:
            self.set_num_frames(len(self.input_files))
        if self.labels_enabled:
            self.set_num_frames(len(self.label_files))

    def set_num_frames(self, num_files: int) -> None:
        """Update the number of frames available in the dataset.

        Args:
            num_files (int): The number of files available for a given sensor.
        """
        if self.num_frames > 0:
            self.num_frames = min(self.num_frames, num_files)
        else:
            self.num_frames = num_files

    ####################################################################
    # handling input data
    ####################################################################
    def import_input_data(self) -> None:
        """Import input files from the dataset folder."""
        path = os.path.join(self.dataset_path, self.input_folder)

        if os.path.isdir(path):
            self.input_files = sorted(os.listdir(path))
            self.inputs_enabled = True
            print("found {} input samples".format(len(self.input_files)))
        else:
            print("did not find input samples")

    def get_input_data(self, idx: int) -> np.ndarray:
        """Get saved input samples.

        Args:
            idx (int): The index of the input data sample.

        Returns:
            np.ndarray: Matrix of input data for the given index.
        """
        assert self.inputs_enabled, "No inputs dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.input_folder,
            self.input_files[idx])

        inputs = np.load(path)

        return inputs

    ####################################################################
    # handling label data
    ####################################################################
    def import_label_data(self) -> None:
        """Import label files from the dataset folder."""
        path = os.path.join(self.dataset_path, self.label_folder)

        if os.path.isdir(path):
            self.label_files = sorted(os.listdir(path))
            self.labels_enabled = True
            print("found {} label samples".format(len(self.label_files)))
        else:
            print("did not find label samples")

    def get_label_data(self, idx: int) -> np.ndarray:
        """Get saved label samples.

        Args:
            idx (int): The index of the label data sample.

        Returns:
            np.ndarray: N-element array of N inputs where 1 indicates the input
                is valid and 0 indicates the input is invalid.
        """
        assert self.labels_enabled, "No labels dataset loaded"

        path = os.path.join(
            self.dataset_path,
            self.label_folder,
            self.label_files[idx])

        labels = np.load(path)

        return labels
