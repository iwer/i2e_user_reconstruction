FILE=paper

all: clean pdf

pdf: $(FILE).pdf

$(FILE).pdf:
	xelatex $(FILE)
	make bibtex
	xelatex $(FILE)
	latex_count=5 ; \
	# from doxygen latex template
	while egrep -s 'Rerun (LaTeX|to get cross-references right)' $(FILE).log && [ $$latex_count -gt 0 ] ;\
	    do \
	      @echo "Rerunning latex...." ;\
	      xelatex $(FILE) ;\
	      latex_count=`expr $$latex_count - 1` ;\
	    done
	#makeindex $(FILE).idx
	xelatex $(FILE)

bibtex:
	bibtex $(FILE)

clean:	
	rm -f $(FILE).snm $(FILE).toc $(FILE).nav $(FILE).out $(FILE).pdf $(FILE).bbl