set -e

LLVM_VERSION=${1:-"none"}

if [ "${LLVM_VERSION}" = "none" ]; then
    echo "No LLVM version specified"
    exit 1
fi

# Cleanup temporary directory and associated files when exiting the script.
cleanup() {
    EXIT_CODE=$?
    set +e
    if [[ -n "${TMP_DIR}" ]]; then
        echo "Executing cleanup of tmp files"
        rm -Rf "${TMP_DIR}"
    fi
    exit $EXIT_CODE
}
trap cleanup EXIT


echo "Installing LLVM..."
mkdir -p /opt/llvm

architecture=$(dpkg --print-architecture)
case "${architecture}" in
    arm64)
        ARCH=ARM64 ;;
    amd64)
        ARCH=X64 ;;
    *)
        echo "Unsupported architecture ${architecture}."
        exit 1
        ;;
esac

LLVM_BINARY_NAME="LLVM-${LLVM_VERSION}-Linux-${ARCH}.tar.xz"
LLVM_CHECKSUM_NAME="${LLVM_BINARY_NAME}.sig"
LLVM_VERSION_MAJOR=$(echo "${LLVM_VERSION}" | cut -d '.' -f 1)
TMP_DIR=$(mktemp -d -t llvm-XXXXXXXXXX)

echo "${TMP_DIR}"
cd "${TMP_DIR}"

curl -sSL "https://releases.llvm.org/release-keys.asc" -O
curl -sSL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LLVM_VERSION}/${LLVM_BINARY_NAME}" -O
curl -sSL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LLVM_VERSION}/${LLVM_CHECKSUM_NAME}" -O

gpg --no-default-keyring --keyring ./keyring.kbx --import release-keys.asc
gpg --no-default-keyring --keyring ./keyring.kbx --verify "${LLVM_CHECKSUM_NAME}" "${LLVM_BINARY_NAME}"

if [ $? -ne 0 ]; then
    echo "Checksum verification failed"
    exit 1
fi

mkdir -p /opt/llvm
tar -xvf "${LLVM_BINARY_NAME}" -C /opt/llvm --strip-components=1

for f in /opt/llvm/bin/*; do 
    ln -s -f "$f" "/usr/local/bin/$(basename $f)"
    ln -s -f "$f" "/usr/local/bin/$(basename $f)-${LLVM_VERSION_MAJOR}"
done