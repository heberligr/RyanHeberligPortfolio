"""Shared test fixtures."""

import copy

import pytest

from segslicer.config import load_machine


@pytest.fixture
def machine():
    """A fresh copy of the default machine.yaml for each test (safe to modify)."""
    return copy.deepcopy(load_machine())
