# Security

This module drives a physical robot arm. Take anything that could bypass its
safety interlocks or release signing seriously.

## Reporting a vulnerability

Report vulnerabilities privately via GitHub:
https://github.com/waypointos/waypoint-so100/security/advisories/new

Do not open a public issue for a security problem. You should hear back
within a week.

## Verifying releases

Every release ships a `.raw` module image signed with cosign keyless
signing from this repository's release workflow. Verify with:

```
cosign verify-blob \
  --bundle so100-<version>.raw.cosign \
  --certificate-oidc-issuer https://token.actions.githubusercontent.com \
  --certificate-identity-regexp '^https://github.com/waypointos/waypoint-so100/\.github/workflows/release\.yml@refs/tags/v' \
  so100-<version>.raw
```
