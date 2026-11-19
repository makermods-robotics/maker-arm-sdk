from maker_arm import _product


def test_identity():
    assert (_product.PACKAGE, _product.CLI_NAME, _product.DIST_NAME) == ("maker_arm", "maker-arm", "maker-arm")
    assert _product.DEFAULT_PROFILE.exists() and _product.DEFAULT_STAR_MAPPING.exists() and _product.URDF.exists()
