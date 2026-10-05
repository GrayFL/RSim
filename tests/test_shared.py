import os

import numpy as np

from rsim import Frame
from rsim.transport.shared import SharedStore, decode


def test_readonly_mapping_survives_history_eviction_and_reuses_inode(tmp_path):
    store = SharedStore(tmp_path / "source", history=1)
    frame = Frame(np.arange(100), 123, "test", sequence=1)
    descriptor = store.put(frame)
    a = decode(descriptor["data"], store.directory)
    b = decode(descriptor["data"], store.directory)
    assert not a.flags.writeable and not b.flags.writeable
    # Two virtual mappings refer to the same physical file pages.
    inode = os.stat(a.filename).st_ino
    assert inode == os.stat(b.filename).st_ino
    downstream = SharedStore(tmp_path / "downstream")
    forwarded = downstream.put(Frame(a, 123, "test"))
    assert os.stat(forwarded["data"]["path"]).st_ino == inode
    store.put(Frame(np.ones(100), 456, "test"))
    assert not os.path.exists(a.filename)
    np.testing.assert_array_equal(a, np.arange(100))
    np.testing.assert_array_equal(b, a)
    downstream.close()
    np.testing.assert_array_equal(a, np.arange(100))


def test_computation_into_shared_output_has_no_payload_copy(tmp_path):
    store = SharedStore(tmp_path)
    result = store.allocate((1000,), np.float64)
    np.multiply(np.arange(1000), 2, out=result)
    inode = os.stat(result.filename).st_ino
    descriptor = store.put(Frame(result, 42, "test"))
    assert os.stat(descriptor["data"]["path"]).st_ino == inode
    read = decode(descriptor["data"], tmp_path)
    assert not read.flags.writeable
    assert not result.flags.writeable
    np.testing.assert_array_equal(read, np.arange(1000) * 2)


def test_worker_history_and_export_use_the_same_promoted_array(tmp_path):
    store = SharedStore(tmp_path)
    original = np.arange(100)
    prepared = store.prepare({"a": original, "b": original})
    assert prepared["a"] is prepared["b"]
    inode = os.stat(prepared["a"].filename).st_ino
    descriptor = store.put(Frame(prepared, 42, "test"))
    assert os.stat(descriptor["data"]["items"]["a"]["path"]).st_ino == inode
    assert os.stat(descriptor["data"]["items"]["b"]["path"]).st_ino == inode
