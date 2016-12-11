PROJECT=orb-4
EAGLE_BIN=/opt/eagle-7.7.0/bin/eagle

FAB_DIR=$(PROJECT)-fab
PRINT_DIR=ps

GERBER_FILES=$(FAB_DIR)/$(PROJECT)-sol-bot-copper.grb.sol \
             $(FAB_DIR)/$(PROJECT)-sol-bot-stop.grb.sts \
             $(FAB_DIR)/$(PROJECT)-sol-bot-silk.grb.pls \
	     $(FAB_DIR)/$(PROJECT)-cmp-top-copper.grb.cmp \
             $(FAB_DIR)/$(PROJECT)-cmp-top-stop.grb.stc \
             $(FAB_DIR)/$(PROJECT)-cmp-top-silk.grb.plc \
             $(FAB_DIR)/$(PROJECT)-fab-doc.grb
EXCELLON_FILES=$(FAB_DIR)/$(PROJECT).drd

EPS_FILES=$(PRINT_DIR)/$(PROJECT)-sol-bot-copper.eps \
	  $(PRINT_DIR)/$(PROJECT)-cmp-top-copper.eps \
          $(PRINT_DIR)/$(PROJECT)-cmp-top-silk.eps \
          $(PRINT_DIR)/$(PROJECT)-sol-bot-silk.eps
PS_PANELS=$(PRINT_DIR)/$(PROJECT)-sol-bot-copper.panel.ps \
	  $(PRINT_DIR)/$(PROJECT)-cmp-top-copper.panel.ps \
	  $(PRINT_DIR)/$(PROJECT)-cmp-top-silk.panel.ps \
	  $(PRINT_DIR)/$(PROJECT)-cmp-top-silk.panel-mirror.ps
PDF_PANELS=$(PRINT_DIR)/$(PROJECT)-sol-bot-copper.panel.pdf \
	   $(PRINT_DIR)/$(PROJECT)-cmp-top-copper.panel.pdf \
	   $(PRINT_DIR)/$(PROJECT)-cmp-top-silk.panel.pdf \
	   $(PRINT_DIR)/$(PROJECT)-cmp-top-silk.panel-mirror.pdf \
	   $(PRINT_DIR)/$(PROJECT)-sol-bot-silk.panel.pdf \
	   $(PRINT_DIR)/$(PROJECT)-sol-bot-silk.panel-mirror.pdf

SCHEMATIC=$(PRINT_DIR)/$(PROJECT)-schematic.pdf
FAB_DOC=$(PRINT_DIR)/$(PROJECT)-fab-doc.pdf
ASM_DOC=$(PRINT_DIR)/$(PROJECT)-assembly-doc.pdf

CAM_PROCESS=$(EAGLE_BIN) -N+ -c+ -X
GERB_CAM=$(CAM_PROCESS) -dGERBER_RS274X
EXCELLON_CAM=$(CAM_PROCESS) -dEXCELLON -E0.025
EPS_CAM=$(CAM_PROCESS) -dEPS

# Gerber mirroing on solder side
GRB_MIRROR=-m-

# Mirror for EPS depending on if toner towards board.
# '-' for toner transfer and transparent photo exposure film.
EPS_SOL_MIRROR=-m-

# Mirror for component side:
# '-' to be readable
# '+' for toner transfer
EPS_CMP_MIRROR=-m-

all : gerber excellon fab-clean postscript

excellon : prep $(EXCELLON_FILES)
gerber : prep $(GERBER_FILES)
postscript : prep $(EPS_FILES) $(SCHEMATIC) $(FAB_DOC) $(ASM_DOC)

clean :
	rm -f $(GERBER_FILES) $(EXCELLON_FILES) \
	      $(EPS_FILES) $(PS_PANELS) $(PDF_PANELS) \
	      $(FAB_DOC) $(ASM_DOC) $(SCHEMATIC)
	if [ -e $(FAB_DIR) ] ; then rmdir $(FAB_DIR); fi
	if [ -e $(PRINT_DIR) ] ; then rmdir $(PRINT_DIR); fi

# remove files, that are not needed by Olimex
fab-clean:
	rm -f $(FAB_DIR)/*.dri $(FAB_DIR)/*.gpi

prep :
	mkdir -p $(FAB_DIR) ps

%.drd: $(PROJECT).brd
	$(EXCELLON_CAM) -o$@ $(PROJECT).brd drills holes

# schematic
%-schematic.eps: $(PROJECT).sch
	$(EPS_CAM) -o$@ -f- $(PROJECT).sch nets busses symbols names values

# solder side:

%-sol-bot-copper.grb.sol: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(GRB_MIRROR) $(PROJECT).brd pad via bottom

%-sol-bot-copper.eps: $(PROJECT).brd
	$(EPS_CAM) -o$@ $(EPS_MIRROR) -f- $(PROJECT).brd pad via bottom dimension

%-sol-bot-stop.grb.sts: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(GRB_MIRROR) $(PROJECT).brd bstop

%-sol-bot-silk.grb.pls: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(GRB_MIRROR) $(PROJECT).brd dimension bplace bnames

%-sol-bot-silk.eps: $(PROJECT).brd
	$(EPS_CAM) -o$@ $(EPS_MIRROR) -f- $(PROJECT).brd dimension bplace bnames

# component side:

%-cmp-top-copper.grb.cmp: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(GRB_CMP_MIRROR) $(PROJECT).brd pad via top

%-cmp-top-copper.eps: $(PROJECT).brd
	$(EPS_CAM) -o$@ $(EPS_CMP_MIRROR) -f- $(PROJECT).brd pad via top dimension

%-cmp-top-stop.grb.stc: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(GRB_CMP_MIRROR) $(PROJECT).brd tstop

%-cmp-top-silk.grb.plc: $(PROJECT).brd
	$(GERB_CAM) -o$@ $(PROJECT).brd $(GRB_CMP_MIRROR) dimension tplace tnames

%-fab-doc.grb : $(PROJECT).brd
	$(GERB_CAM) -o$@ $(PROJECT).brd dimension document tplace tDocu pad drills milling measures

%-cmp-top-silk.eps: $(PROJECT).brd
	$(EPS_CAM) -o$@ $(EPS_CMP_MIRROR) $(PROJECT).brd dimension tplace tnames

%-fab-doc.eps: $(PROJECT).brd
	$(EPS_CAM) -s 3 -r+ -o$@ $(PROJECT).brd dimension document tplace tDocu pad drill milling measures

%-assembly-doc.eps: $(PROJECT).brd
	$(EPS_CAM) -s 5 -r+ -o$@ $(PROJECT).brd dimension tplace tDocu tOrigins tNames Pads

# Its easier to center the manual drill if the holes are small.
%.smallhole-eps : %.eps
	sed 's|/h {|/h { pop 3000|' < $< > $@

%.panel.ps : %.smallhole-eps ~/bin/panelize.sh
	~/bin/panelize.sh $< > $@

%.panel-mirror.ps : %.eps ~/bin/panelize.sh
	~/bin/panelize.sh -m $< > $@

%.pdf : %.ps
	ps2pdf $< $*.pdf

%.pdf : %.eps
	ps2pdf $< $*.pdf

%.png : %.pov
	povray +L ~/eagle/ulp/eagle3d/povray/ +L /usr/lib/cinelerra/fonts/ +W1024 +H768 +Q11 +A0.05 $<
