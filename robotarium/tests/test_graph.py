import robotarium
from numpy.testing import assert_allclose

def test_complete_gl():
    n = 5
    graph = robotarium.graph.complete_gl(n)
    print(graph)
    assert graph.sum() == 0
    assert_allclose(graph.diagonal(), n - 1)
    assert_allclose(graph.sum(axis=1), 0)
