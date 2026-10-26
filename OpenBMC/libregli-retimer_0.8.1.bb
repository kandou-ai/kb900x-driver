DESCRIPTION = "The library to communicate with Kandou kb900x retimer."
SUMMARY = "kb900x Retimer Library"
HOMEPAGE = "https://www.kandou.ai/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/Apache-2.0;md5=89aea4e17d99a7cacdbeed46a0096b10"

# Set default value (can be overridden in a .bbappend or local.conf)
KB900X_BIC_COMMUNICATION ??= "False"
KB900X_PRODUCT ??= "KB9003"

S = "${WORKDIR}/git"
SRC_URI = "git://github.com/kandou-ai/kb900x-driver.git;branch=main;protocol=https"
SRCREV = "${AUTOREV}"

inherit meson pkgconfig

DEPENDS += ""

RDEPENDS:${PN} += ""

python __anonymous() {
    bic_comm = d.getVar('KB900X_BIC_COMMUNICATION')
    product = d.getVar('KB900X_PRODUCT')
    deps = d.getVar('DEPENDS') or ""
    rdeps = d.getVar('RDEPENDS') or ""
    meson_opts = d.getVar("EXTRA_OEMESON") or ""
    if bic_comm == "True":
        deps += " libbic"
        rdeps += " libbic"
        meson_opts += " -Dbic_communication=true"
    if product == "KB9003":
        meson_opts += " -Dproduct=KB9003"
    elif product == "KB9002":
        meson_opts += " -Dproduct=KB9002"
    else:
        raise ValueError("Unsupported product: {}".format(product))
    d.setVar('DEPENDS', deps)
    d.setVar('RDEPENDS', rdeps)
    d.setVar("EXTRA_OEMESON", meson_opts)
}
