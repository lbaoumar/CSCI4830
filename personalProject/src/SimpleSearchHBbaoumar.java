import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import datamodel.Student;
import util.Info;
import util.UtilDBbaoumar;

@WebServlet("/SimpleSearchHBbaoumar")
public class SimpleSearchHBbaoumar extends HttpServlet implements Info {
   private static final long serialVersionUID = 1L;

   public SimpleSearchHBbaoumar() {
      super();
   }

   protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
	   String keyword = "%";
	  // keyword = request.getParameter("keyword").trim();
	   
      response.setContentType("text/html");
      PrintWriter out = response.getWriter();
      String title = "Database Result";
      String docType = "<!doctype html public \"-//w3c//dtd html 4.0 transitional//en\">\n"; //
      out.println(docType + //
            "<html>\n" + //
            "<head><title>" + title + "</title></head>\n" + //
            "<body bgcolor=\"#f0f0f0\">\n" + //
            "<h1 align=\"center\">" + title + "</h1>\n");
      out.println("<ul>");

      List<Student> listEmployees = null;
      if (keyword != null && !keyword.isEmpty()) {
         listEmployees = UtilDBbaoumar.listEmployees(keyword);
      } else {
         listEmployees = UtilDBbaoumar.listStudents();
      }
      display(listEmployees, out);
      out.println("</ul>");
      out.println("<a href=/" + projectName + "/" + searchWebName + ">Search Data</a> <br>");
      out.println("</body></html>");
   }

   void display(List<Student> listEmployees, PrintWriter out) {
      for (Student employee : listEmployees) {
         System.out.println("[DBG] " + employee.getId() + ", " //
        		  + employee.getFirstName() + ", " //
                  + employee.getLastName() + ", "
                  + employee.getNUID() + ", "
                  + employee.getemail());

         out.println("<li>" + employee.getId() + ", " //
        		  + employee.getFirstName() + ", " //
                  + employee.getLastName() + ", "
                  + employee.getNUID() + ", "
                  + employee.getemail());
      }
   }

   protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
      doGet(request, response);
   }
}
