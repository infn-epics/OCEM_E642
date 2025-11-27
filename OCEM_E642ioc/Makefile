TOP = ..
include $(TOP)/configure/CONFIG

# Directories to be built, in any order.
# You can replace these wildcards with an explicit list
DIRS += $(wildcard db* *Db*)
DIRS += $(wildcard src* *Src*)


# If the build order matters, add dependency rules like this,
# which specifies that xxxSrc must be built after src:
#xxxSrc_DEPEND_DIRS += src

include $(TOP)/configure/RULES_DIRS