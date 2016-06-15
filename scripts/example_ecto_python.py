import ecto
import numpy as np
class ClusterDetector(ecto.Cell):
    def declare_params(self, params):
        params.declare("n", "Max number of clusters.", 10)

    def declare_io(self, params, inputs, outputs):
        outputs.declare("clusters", "Clusters output. list of tuples", [])

    def process(self, inputs, outputs):
        clusters = []
        for i in range(int(np.random.uniform(0, self.params.n))):
            clusters.append( (i, 'c%d'%i) )
        outputs.clusters = clusters

class ClusterPrinter(ecto.Cell):
    def declare_io(self, params, inputs, outputs):
        inputs.declare("clusters", "Clusters input")

    def process(self, inputs, outputs):
        print "Clusters: ",
        for c in inputs.clusters:
            print c,
        print "\n"

def app():
    cd = ClusterDetector(n=20)
    cp = ClusterPrinter()
    plasm = ecto.Plasm()
    plasm.connect(cd['clusters'] >> cp['clusters'])
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=3)

if __name__ == "__main__":
    app()