# Publishing a previously tested package

The `publish-tested-package.yml` workflow can publish the vendordep artifact
from a successful pull-request run when its merge commit and the release tag
have identical source trees and parents. It checks both platform and consumer
jobs, clean build provenance, the pinned Base revision, descriptor and package
checksums before deploying. Deployed consumer configurations run separately
with a three-minute timeout per platform job.
