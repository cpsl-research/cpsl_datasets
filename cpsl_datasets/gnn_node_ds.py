import os
import numpy as np

from cpsl_datasets._base_training_ds import _BaseTrainingDS

class GnnNodeDS(_BaseTrainingDS):
    """Dataset class for handling node and label data for GNNs."""

    def __init__(self,
                 dataset_path: str,
                 node_folder: str = "nodes",
                 label_folder: str = "labels",
                 ) -> None:
        """Initialize the node dataset.

        Args:
            dataset_path (str): Path to the dataset directory.
            node_folder (str): Name of the folder containing node data.
            label_folder (str): Name of the folder containing label data.
        """
        super().__init__(
            dataset_path=dataset_path,
            input_folder=node_folder,
            label_folder=label_folder
        )

    @property
    def node_folder(self) -> str:
        """Get the node folder name.

        Returns:
            str: The node folder name.
        """
        return self.input_folder

    @node_folder.setter
    def node_folder(self, value: str) -> None:
        """Set the node folder name.

        Args:
            value (str): The new node folder name.
        """
        self.input_folder = value

    @property
    def nodes_enabled(self) -> bool:
        """Check if node data is enabled.

        Returns:
            bool: True if nodes are enabled, False otherwise.
        """
        return self.inputs_enabled

    @nodes_enabled.setter
    def nodes_enabled(self, value: bool) -> None:
        """Set nodes enabled flag.

        Args:
            value (bool): True if nodes are enabled, False otherwise.
        """
        self.inputs_enabled = value

    @property
    def node_files(self) -> list:
        """Get the list of node files.

        Returns:
            list: List of node file names.
        """
        return self.input_files

    @node_files.setter
    def node_files(self, value: list) -> None:
        """Set the list of node files.

        Args:
            value (list): List of node file names.
        """
        self.input_files = value

    def import_node_data(self) -> None:
        """Import node files from the dataset folder."""
        self.import_input_data()

    def get_node_data(self, idx: int) -> np.ndarray:
        """Get saved node samples.

        Args:
            idx (int): The index of the node data sample.

        Returns:
            np.ndarray: NxM array of N nodes with M properties per Node.
        """
        return self.get_input_data(idx)