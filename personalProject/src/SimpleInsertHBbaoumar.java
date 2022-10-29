import java.io.IOException;
import java.io.PrintWriter;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import util.Info;
import util.UtilDBbaoumar;

@WebServlet("/SimpleInsertHBbaoumar")
public class SimpleInsertHBbaoumar extends HttpServlet implements Info {
   private static final long serialVersionUID = 1L;

   public SimpleInsertHBbaoumar() {
      super();
   }

   protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
      String firstName = request.getParameter("FirstName").trim();
      String lastName = request.getParameter("LastName").trim();
      String NUID = request.getParameter("studentNuid").trim();
      String email = request.getParameter("Email").trim();
      UtilDBbaoumar.createStudents(firstName, lastName, NUID, email);

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
      out.println("<li> Fist Name: " + firstName);
      out.println("<li> Last Name: " + lastName);
      out.println("<li> NUID: " + NUID);
      out.println("<li> Email: " + email);
      out.println("</ul>");
      out.println("<a href=/" + projectName + "/" + thanksforsigning + ">Next</a> <br>");
      out.println("</body></html>");
   }

   protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
      doGet(request, response);
   }
}
