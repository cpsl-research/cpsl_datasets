import importlib.util

import pytest


def test_import_package():
    import cpsl_datasets  # noqa: F401


def test_import_gnn_node_ds():
    from cpsl_datasets.gnn_node_ds import GnnNodeDS  # noqa: F401


def test_import_map_handler():
    from cpsl_datasets.map_handler import MapHandler  # noqa: F401


def test_import_base_training_ds():
    import cpsl_datasets._base_training_ds  # noqa: F401


@pytest.mark.skipif(
    importlib.util.find_spec("geometries") is None,
    reason="geometries not installed (uv sync --extra athena)",
)
def test_import_athena_uav_ds():
    import cpsl_datasets.athena_uav_ds  # noqa: F401
