SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Clone repositories
clone_repositories() {
    cd "$WORKSPACE_ROOT"
    if [ ! -d "src" ]; then
        mkdir -p src
        vcs import src <autoware.repos
        vcs import src <extra-packages.repos
    else
        echo "Source directory already exists. Updating repositories..."
        vcs import src <autoware.repos
        vcs import src <extra-packages.repos
        vcs pull src
    fi
}


clone_repositories