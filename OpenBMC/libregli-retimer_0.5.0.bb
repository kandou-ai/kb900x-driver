DESCRIPTION = "The library to communicate with Kandou kb900x retimer."
SUMMARY = "kb900x Retimer Library"
HOMEPAGE = "https://www.kandou.ai/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/Apache-2.0;md5=89aea4e17d99a7cacdbeed46a0096b10"

# Set default value (can be overridden in a .bbappend or local.conf)
KB900X_BIC_COMMUNICATION ??= "False"

S = "${WORKDIR}/sources"
UNPACKDIR="${S}"

SRC_URI = "git://github.com/kandou-ai/kb900x-driver.git;branch=main"
SRCREV = "${AUTOREV}"

inherit meson pkgconfig

DEPENDS += ""

RDEPENDS:${PN} += ""

python __anonymous() {
    bic_comm = d.getVar('KB900X_BIC_COMMUNICATION')
    if bic_comm == "True":
        deps = d.getVar('DEPENDS') or ""
        rdeps = d.getVar('RDEPENDS') or ""
        meson_opts = d.getVar("EXTRA_OEMESON") or ""

        deps += " libbic"
        rdeps += " libbic"
        meson_opts += " -Dbic_communication=true"

        d.setVar('DEPENDS', deps)
        d.setVar('RDEPENDS', rdeps)
        d.setVar("EXTRA_OEMESON", meson_opts)
}
